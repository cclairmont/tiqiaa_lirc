/***************************************************************************
** tiqiaa.c **********************************************************
****************************************************************************
*  Userspace (libusb) driver for Tiqiaa Tview Remote.
*
*  Copyright (C) 2020 Connor Clairmont <clairmont.connor@gmail.com>
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Library General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
*/
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <glob.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <lirc_driver.h>

#define CODE_BYTES 5
#define LIBUSB_TIMEOUT (1000 * 60)

#pragma pack(1)

static const uint8_t cmd_unknown = 'H';
static const uint8_t cmd_version = 'V';
static const uint8_t cmd_idle_mode = 'L';
static const uint8_t cmd_send_mode = 'S';
static const uint8_t cmd_recv_mode = 'R';
static const uint8_t cmd_data = 'D';
static const uint8_t cmd_output = 'O';
static const uint8_t cmd_cancel = 'C';

static const uint8_t state_idle = 3;
static const uint8_t state_send = 9;
static const uint8_t state_recv = 19;

static const int max_usb_frag_size = 56;
static const int max_usb_packet_size = 1024;
static const int max_usb_packet_idx = 15;
static const int max_cmd_id = 0x7F;
static const uint16_t pack_start_sign = 0x5453; //"ST"
static const uint16_t pack_end_sign = 0x4E45; //"EN"
static const uint8_t write_pipe_id = 1;
static const uint8_t read_pipe_id = 0x81;
static const uint8_t write_report_id = 2;
static const uint8_t read_report_id = 1;
static const uint16_t cmd_reply_wait_timeout = 500;
static const uint16_t ir_reply_wait_timeout = 2000;

static const int nec_pulse_size = 1125; //562.5 mks
static const int ir_send_tick_size = 32; //16 mks
static const int max_ir_send_block_size = 127; //ticks

static int is_waiting_cmd_reply;
static int is_cmd_reply_received;
static uint8_t wait_cmd_id;
static uint8_t wait_cmd_type;
static uint8_t device_state;

static const logchannel_t logchannel = LOG_DRIVER;

typedef struct {
	u_int16_t	vendor;
	u_int16_t	product;
} usb_device_id;

typedef struct {
	uint8_t report_id;
	uint8_t frag_size;
	uint8_t packet_idx;
	uint8_t frag_count;
	uint8_t frag_idx;
} tiq_report2header;

typedef struct {
	uint16_t start_sign;
	uint8_t cmd_id;
	uint8_t cmd_type;
	uint16_t end_sign;
} tiq_send_cmd_pack;

typedef struct {
	uint16_t start_sign;
	uint8_t cmd_id;
	uint8_t cmd_type;
	uint8_t ir_freq_id;
} tiq_send_ir_pack_header;

typedef struct {
	uint8_t version_char;
	uint8_t version_int;
	uint8_t version_guid[0x24];
	uint8_t state;
} tiq_version_pack;

typedef struct {
	uint8_t *buf;
	int size;
	int pulse_time;
	int sender_time;
} tiq_ir_write_data;

static int tiq_init(void);
static int tiq_deinit(void);
static char* tiq_rec(struct ir_remote* remotes);
static int tiq_send(struct ir_remote* remote, struct ir_ncode* code);
static struct libusb_device* find_usb_device(void);
static int find_device_endpoints(struct libusb_device* dev);
static char device_path[10000] = {0};
static int drvctl_func(unsigned int cmd, void* arg);
static uint8_t get_cmd_id(void);
static int send_cmd(uint8_t cmd_type, uint8_t cmd_id);
static int set_idle_mode(void);
static int write_ir_nec_signal(ir_code code, uint8_t *out_buf);
static void write_ir_nec_signal_pulse(tiq_ir_write_data* ir_data, int pulse_count, int is_set);
static int send_ir(int freq, void *buffer, int buf_size);
static int send_ir_cmd(int freq, void *buffer, int buf_size, uint8_t cmd_id);
static int send_report_2(void *data, int size);
static int write_pulse(uint8_t *buf, int buf_size, int is_on, lirc_t duration);
static void usb_read_loop(int fd);
static int rec_response(void *buf, int buf_size);

const struct driver hw_tiqlibusb = {
	.name		= "tiqlibusb",
	.device		= NULL,
	.fd		= -1,
	.features	= LIRC_CAN_REC_LIRCCODE | LIRC_CAN_SEND_PULSE,
	.send_mode	= LIRC_MODE_PULSE,
	.rec_mode	= LIRC_MODE_LIRCCODE,
	.code_length	= CODE_BYTES * CHAR_BIT,
	.init_func	= tiq_init,
	.deinit_func	= tiq_deinit,
	.open_func	= default_open,
	.close_func	= default_close,
	.send_func	= tiq_send,
	.rec_func	= NULL, //tiq_rec,
	.decode_func	= receive_decode,
	.drvctl_func	= drvctl_func,
	.readdata	= NULL,
	.api_version	= 3,
	.driver_version = "0.10.0",
	//.info		= "See file://" PLUGINDOCS "/tiqlibusb.html",
	.device_hint    = "auto",
};

const struct driver* hardwares[] = { &hw_tiqlibusb, (const struct driver*)NULL };

/* table of compatible remotes */
static usb_device_id usb_remote_id_table[] = {
	{ 0x10C4, 0x8468 },
	{ 0x45E, 0x8468 },
	{ 0,	  0	 }      /* Terminating entry */
};

static const int tiq_ir_freq_table_size = 30;
static const int tiq_ir_freq_table[30] = {
38000, 37900, 37917, 36000, 40000, 39700, 35750, 36400, 36700, 37000, 
37700, 38380, 38400, 38462, 38740, 39200, 42000, 43600, 44000, 33000,
33500, 34000, 34500, 35000, 40500, 41000, 41500, 42500, 43000, 45000};

static struct libusb_device_handle *dev_handle = NULL;
static struct libusb_endpoint_descriptor dev_ep_in;
static struct libusb_endpoint_descriptor dev_ep_out;
static uint8_t cmd_id = 1;
static int packet_idx = 0;
static pid_t child = -1;
static int read_active = 1;

/* returns 1 if the given device should be used, 0 otherwise */
static int is_device_ok(uint16_t vendor,  uint16_t product)
{
	usb_device_id* d;

	for (d = usb_remote_id_table; d->vendor; d++) {
		if ((vendor == d->vendor) && (product == d->product))
			return 1;
	}
	return 0;
}

static int is_usb_device_ok(struct libusb_device* d)
{
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(d, &desc);
	return is_device_ok(desc.idVendor, desc.idProduct);
}

static int drvctl_func(unsigned int cmd, void* arg)
{
	switch (cmd) {
	case DRVCTL_GET_DEVICES:
		return drv_enum_usb((glob_t*) arg, is_device_ok);
	case DRVCTL_FREE_DEVICES:
		drv_enum_free((glob_t*) arg);
		return 0;
	default:
		return DRV_ERR_NOT_IMPLEMENTED;
	}
}

/* initialize driver -- returns 1 on success, 0 on error */
static int tiq_init(void)
{
	struct libusb_device* usb_dev;
    int pipe_fd[2] = { -1, -1 };

	log_trace("initializing USB transmitter");

	rec_buffer_init();

	/* A separate process will be forked to read data from the USB
	 * receiver and write it to a pipe. drv.fd is set to the readable
	 * end of this pipe. */
	if (pipe(pipe_fd) != 0) {
		log_perror_err("couldn't open pipe");
		return 0;
	}
	drv.fd = pipe_fd[0];
    usb_dev = find_usb_device();
    if (!usb_dev) {
		log_error("couldn't find a compatible USB device");
		return 0;
	}
    
    snprintf(device_path, sizeof(device_path),
		 "/dev/bus/usb/%03d/%03d\n",
		 libusb_get_bus_number(usb_dev),
         libusb_get_device_address(usb_dev));
	drv.device = device_path;
	if (usb_dev == NULL) {
		log_error("couldn't find a compatible USB device");
		return 0;
	}

	if (!find_device_endpoints(usb_dev)) {
		log_error("couldnt find device endpoints");
		return 0;
	}
    
    libusb_open(usb_dev, &dev_handle);
	if (dev_handle == NULL) {
		log_perror_err("couldn't open USB receiver");
		goto fail;
	}

	if (libusb_claim_interface(dev_handle, 0) != 0) {
		log_perror_err("couldn't claim USB interface");
		goto fail;
	}
    
    errno = 0;
	if (!send_cmd(cmd_idle_mode, get_cmd_id()) ||
        !send_cmd(cmd_send_mode, get_cmd_id())) {
		log_error("couldn't initialize USB receiver: %s", errno ? strerror(errno) : "short write");
		goto fail;
	}
    
    snprintf(device_path, sizeof(device_path),
		 "/dev/bus/usb/%03d/%03d\n",
		 libusb_get_bus_number(usb_dev),
         libusb_get_device_address(usb_dev));
	drv.device = device_path;
    
    log_debug("tiqiaa: using device: %s", device_path);
    // child = fork();
	// if (child == -1) {
		// log_perror_err("couldn't fork child process");
		// goto fail;
	// } else if (child == 0) {
		// usb_read_loop(pipe_fd[1]);
	// }
	return 1;

fail:
	if (dev_handle) {
		libusb_close(dev_handle);
		dev_handle = NULL;
	}
	if (pipe_fd[0] >= 0)
		close(pipe_fd[0]);
	if (pipe_fd[1] >= 0)
		close(pipe_fd[1]);
	return 0;
}

static int tiq_deinit(void) {
	int err = 0;
    log_info("Deinit");
    
    set_idle_mode();

	if (dev_handle) {
		libusb_close(dev_handle);
		dev_handle = NULL;
	}

	if (drv.fd >= 0) {
		if (close(drv.fd) < 0)
			err = 1;
		drv.fd = -1;
	}
    
    if (child > 1) {
		if ((kill(child, SIGTERM) == -1)
		    || (waitpid(child, NULL, 0) == 0))
			err = 1;
	}
	return !err;
}

// static void usb_read_loop(int fd){
	// uint8_t frag_buf[61];
	// uint8_t pack_buf[1024];
	// int pack_size;
	// int frag_size;
	// uint8_t pack_idx;
	// uint8_t frag_count;
	// uint8_t last_frag_idx;
	// tiq_report2header *report_hdr = (tiq_report2header *)frag_buf;
	// uint32_t usb_rx_size;

	// frag_count = 0; //not receiving packet
	// while (read_active){
        // int result = libusb_bulk_transfer(dev_handle, dev_ep_in.bEndpointAddress,
            // frag_buf, 61, &usb_rx_size, 100);
        // log_info("%d", result);
        // if(result) {
            // log_info("Transfer error: %d", result);
            // return;
        // }
		// if (!result){
            // log_info("Received legitimate packet size %d", usb_rx_size);
            // continue;
			// if ((usb_rx_size > sizeof(tiq_report2header)) && // Received frag greater than size of header
                // (report_hdr -> report_id == read_report_id) && // Packet should be read
                // ((uint32_t)(report_hdr -> frag_size + 2) <= usb_rx_size)){ // report_hdr is the start of frag_buf, not sure what the 2 extra bytes are
				// if (frag_count){//adding data to existing packet
					// if ((report_hdr -> packet_idx == pack_idx) && // is it the correct packet?
                        // (report_hdr -> frag_count == frag_count) && // do frag counts match?
                        // (report_hdr -> frag_idx == (last_frag_idx + 1))){ // is it the next frag in the sequence?
						// last_frag_idx++;
					// } else {//wrong fragment - drop packet
						// frag_count = 0;
					// }
				// }
				// if (frag_count == 0){//new packet
					// if ((report_hdr -> frag_count > 0) &&
                        // (report_hdr -> frag_idx == 1)){
						// pack_idx = report_hdr -> packet_idx;
						// frag_count = report_hdr -> frag_count;
						// pack_size = 0;
						// last_frag_idx = 1;
					// }
				// }
				// if (frag_count){
					// frag_size = report_hdr -> frag_size + 2 - sizeof(tiq_report2header);
					// if ((pack_size + frag_size) <= max_usb_packet_size){
						// memcpy(pack_buf + pack_size, frag_buf + sizeof(tiq_report2header), frag_size);
						// pack_size += frag_size;
						// if ((report_hdr -> frag_idx == last_frag_idx) && (pack_size > 6)){
							// if ((*((uint16_t *)(pack_buf)) == pack_start_sign) &&
                                // (*((uint16_t *)(pack_buf + pack_size - 2)) == pack_end_sign)){
								// ProcessRecvPacket(PackBuf + 2, PackSize - 4);
                                // log_info("Received packet");
							// }
						// }
					// } else {//buffer overflow - drop packet
						// frag_count = 0;
					// }
				// }
			// }
		// }
	// }
// }

static char* tiq_rec(struct ir_remote* remotes) {
    if (!rec_buffer_clear()) {
		tiq_deinit();
		return NULL;
	}
	return decode_all(remotes);
}

static int tiq_send(struct ir_remote* remote, struct ir_ncode* code) {
    log_trace("tiqiaa.c: sending, code: %s", code -> name);
    
	uint8_t buf[128];
	int buf_size = 0;
    
    if (!send_buffer_put(remote, code)) {
		log_debug("file.c: Cannot make send_buffer_put");
		return 0;
	}
	for (int i = 0;; ) {
        buf_size = write_pulse(buf, buf_size, 1, send_buffer_data()[i++]);
        if (buf_size < 0) {
            return 0;
        }
		if (i >= send_buffer_length())
			break;
		buf_size = write_pulse(buf, buf_size, 0, send_buffer_data()[i++]);
        if (buf_size < 0) {
            return 0;
        }
	}
	buf_size = write_pulse(buf, buf_size, 0, remote->min_remaining_gap);
    if (buf_size < 0) {
        return 0;
    }
    return send_ir(38000, buf, buf_size);
}

static int write_pulse(uint8_t *buf, int buf_size, int is_on, lirc_t duration) {
    int pulse_length = duration/16;
    while (pulse_length > 127) {
        if (buf_size > 127) {
            log_error("Buffer overflow");
            return -1;
        }
        buf[buf_size] = (128 * is_on) + 127;
        pulse_length -= 127;
        buf_size++;
    }
    if (buf_size > 127) {
        log_error("Buffer overflow");
        return -1;
    }
    buf[buf_size] = (128 * is_on) + pulse_length;
    return ++buf_size;
}

static int send_ir(int freq, void *buffer, int buf_size) {
	// if (device_state != state_send){
		// if (!send_cmd(cmd_send_mode, get_cmd_id()))
            // return 0;
	// }
	// if (device_state != state_send)
        // return 0;
	uint8_t send_ir_cmd_id = get_cmd_id();
	// if (!send_cmd(cmd_output, send_ir_cmd_id))
        // return 0;
	if (send_ir_cmd(freq, buffer, buf_size, send_ir_cmd_id))
		return 1;
	return 0;
}

static int send_ir_cmd(int freq, void *buffer, int buf_size, uint8_t cmd_id) {
	uint8_t pack_buf[1024];
	tiq_send_ir_pack_header *pack_header = (tiq_send_ir_pack_header *) pack_buf;
	uint8_t ir_freq_id;
	int pack_size = sizeof(tiq_send_ir_pack_header);

	if (buf_size < 0)
        return 0;
	if ((buf_size + sizeof(tiq_send_ir_pack_header) + sizeof(uint16_t)) > max_usb_packet_size)
        return 0;
	if (freq > 255){
		ir_freq_id = 0;
		while ((ir_freq_id < tiq_ir_freq_table_size) &&
               (tiq_ir_freq_table[ir_freq_id] != freq))
            ir_freq_id++;
		if (ir_freq_id >= tiq_ir_freq_table_size)
            return 0;
	} else {
		if (freq < tiq_ir_freq_table_size)
            ir_freq_id = freq;
        else
            return 0;
	}
	pack_header -> start_sign = pack_start_sign;
	pack_header -> cmd_type = 'D';
	pack_header -> cmd_id = cmd_id;
	pack_header -> ir_freq_id = ir_freq_id;
	memcpy(pack_buf + pack_size, buffer, buf_size);
	pack_size += buf_size;
	*(uint16_t *)(pack_buf + pack_size) = pack_end_sign;
	pack_size += sizeof(uint16_t);
	return send_report_2(pack_buf, pack_size) && set_idle_mode();
}

/* find a compatible USB receiver and return a usb_device,
 * or NULL on failure. */
static struct libusb_device* find_usb_device(void)
{
	libusb_init(NULL);
    struct libusb_device **usb_list;
    ssize_t list_size = libusb_get_device_list(NULL, &usb_list);
	for (int i = 0; i < list_size; i++) {
      if (is_usb_device_ok(usb_list[i])) {
          return usb_list[i];
      }
    }
	return NULL;            /* no suitable device found */
}

static int find_device_endpoints(struct libusb_device *dev)
{
    struct libusb_device_descriptor ddesc;
	struct libusb_config_descriptor *cdesc;
    struct libusb_interface_descriptor idesc;
    
    libusb_get_device_descriptor(dev, &ddesc);
    libusb_get_config_descriptor(dev, 0, &cdesc);

	if (ddesc.bNumConfigurations != 1) {
        log_error("bNumConfig: %d", ddesc.bNumConfigurations);
		return 0;
    }
	if (cdesc -> bNumInterfaces != 1) {
        log_error("bNumInterfaces: %d", cdesc -> bNumInterfaces);
		return 0;
    }
	if (cdesc -> interface[0].num_altsetting != 1) {
        log_error("num_altsetting: %d", cdesc -> interface[0].num_altsetting);
		return 0;
    }
	idesc = cdesc -> interface[0].altsetting[0];
	if (idesc.bNumEndpoints != 2) {
        log_error("bNumEndpoints: %d", idesc.bNumEndpoints);
		return 0;
    }
	dev_ep_in = idesc.endpoint[0];
	if ((dev_ep_in.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK)
	    != LIBUSB_ENDPOINT_IN) {
        log_error("Endpoint dir mask in");
		return 0;
    }
	if ((dev_ep_in.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
	    != LIBUSB_TRANSFER_TYPE_BULK) {
        log_error("EP in not bulk");
		return 0;
    }

	dev_ep_out = idesc.endpoint[1];
	if ((dev_ep_out.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK)
	    != LIBUSB_ENDPOINT_OUT) {
        log_error("Endpoint dir mask out");
		return 0;
    }
	if ((dev_ep_out.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
	    != LIBUSB_TRANSFER_TYPE_BULK) {
        log_error("EP out not bulk");
		return 0;
    }
    
    log_info("dev_ep_in: 0x%X, dev_ep_out 0x%X", dev_ep_in.bEndpointAddress,
        dev_ep_out.bEndpointAddress);

	return 1;
}

static uint8_t get_cmd_id(void) {
    if (cmd_id < max_cmd_id) {
        cmd_id++;
    } else {
        cmd_id = 1;
    }
    return cmd_id;
}

static int send_report_2(void *data, int size) {
	uint8_t frag_buf[61];
	tiq_report2header *report_hdr = (tiq_report2header *) frag_buf;
	int rd_ptr;
	int frag_idx;
	int frag_size;
	uint32_t usb_tx_size;

	rd_ptr = 0;
	if ((size <= 0) || (size > max_usb_packet_size)) {
        return 0;
    }
	memset(frag_buf, 0, sizeof(frag_buf));
	report_hdr -> report_id = write_report_id;
	report_hdr -> frag_count = size / max_usb_frag_size;
	if ((size % max_usb_frag_size) != 0) {
        report_hdr -> frag_count++;
    }
	packet_idx++;
	if (packet_idx > max_usb_packet_idx) {
        packet_idx = 1;
    }
	report_hdr -> packet_idx = packet_idx;
	frag_idx = 0;
	while (rd_ptr < size){
		frag_idx++;
		report_hdr -> frag_idx = frag_idx;
		frag_size = size - rd_ptr;
		if (frag_size > max_usb_frag_size) {
            frag_size = max_usb_frag_size;
        }
		report_hdr -> frag_size = frag_size + 3;
		memcpy(frag_buf + sizeof(tiq_report2header),
            ((uint8_t *)data) + rd_ptr, frag_size);
		int err = libusb_bulk_transfer(dev_handle, 
            dev_ep_out.bEndpointAddress,
            frag_buf,
            frag_size + sizeof(tiq_report2header),
            &usb_tx_size, 0);
        if (err) {
                log_error("%d", err);
                return 0;
            }
		rd_ptr += frag_size;
	}
	return rec_response(NULL, 0);
}

static int send_cmd(uint8_t cmd_type, uint8_t cmd_id){
	tiq_send_cmd_pack pack;
    uint8_t *plist = (uint8_t *)(&pack);

	pack.start_sign = pack_start_sign;
	pack.cmd_type = cmd_type;
	pack.cmd_id = cmd_id;
	pack.end_sign = pack_end_sign;
	return send_report_2(&pack, sizeof(pack));
}

static int set_idle_mode(void){
	return send_cmd(cmd_idle_mode, get_cmd_id());
}

static int rec_response(void *buf, int buf_size) {
    uint8_t frag_buf[63];
    uint8_t frag_count = 1;
    uint8_t frag_size;
    uint8_t frag_idx = 0;
    tiq_report2header *report_hdr = (tiq_report2header *) frag_buf;
    uint32_t usb_tx_size;
    int pack_size = 0;
    
    while (frag_count - frag_idx > 0) {
        int err = libusb_bulk_transfer(dev_handle, 
            dev_ep_in.bEndpointAddress,
            frag_buf, 63, &usb_tx_size, 500);
        if (err) {
            log_info("Error receiving response: %d", err);
            return 1;
        }
        if (frag_idx == 0) {
            frag_count = report_hdr -> frag_count;
        }
        frag_idx++;
        log_info("Received frag %d of %d, size: %d", frag_idx, frag_count, usb_tx_size);
    }
    return 1;
}
