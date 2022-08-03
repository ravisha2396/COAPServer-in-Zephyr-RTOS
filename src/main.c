/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_coap_server_sample, LOG_LEVEL_DBG);

#include <device.h>
#include <drivers/gpio.h>
#include <devicetree.h>
#include <kernel.h>
#include <stdio.h>

#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <string.h>

#include <net/socket.h>
#include <net/net_mgmt.h>
#include <net/net_ip.h>
#include <net/udp.h>
#include <net/coap.h>
#include <net/coap_link_format.h>
#include <stdlib.h>

#include <sys/printk.h>

#include <fsl_iomuxc.h>
#include <drivers/sensor.h>


#include "net_private.h"
#if defined(CONFIG_NET_IPV6)
#include "ipv6.h"
#endif

#define MAX_COAP_MSG_LEN 256

#define MY_COAP_PORT 5683

#define BLOCK_WISE_TRANSFER_SIZE_GET 2048

#if defined(CONFIG_NET_IPV6)
#define ALL_NODES_LOCAL_COAP_MCAST \
	{ { { 0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xfd } } }

#define MY_IP6ADDR \
	{ { { 0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1 } } }
#endif

#define NUM_OBSERVERS 2

#define NUM_PENDINGS 3

#define MAX_RETRANSMIT_COUNT 2

/* Device declarations */

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

#define SENSOR_1 1
#define SENSOR_2 2


#define RED 				r_led

#define RED_LED_NODE 		DT_NODELABEL(RED)

#define RED_LED_LABEL 		DT_GPIO_LABEL(RED_LED_NODE, gpios)

#define RED_LED_PIN 		DT_GPIO_PIN(RED_LED_NODE, gpios)

#define RED_LED_FLAGS 		DT_GPIO_FLAGS(RED_LED_NODE, gpios)

#define GREEN 				g_led

#define GREEN_LED_NODE 		DT_NODELABEL(GREEN)

#define GREEN_LED_LABEL		DT_GPIO_LABEL(GREEN_LED_NODE, gpios)

#define GREEN_LED_PIN 		DT_GPIO_PIN(GREEN_LED_NODE, gpios)

#define GREEN_LED_FLAGS		DT_GPIO_FLAGS(GREEN_LED_NODE, gpios)

#define BLUE 				b_led

#define BLUE_LED_NODE 		DT_NODELABEL(BLUE)

#define BLUE_LED_LABEL 		DT_GPIO_LABEL(BLUE_LED_NODE, gpios)

#define BLUE_LED_PIN 		DT_GPIO_PIN(BLUE_LED_NODE, gpios)

#define BLUE_LED_FLAGS 		DT_GPIO_FLAGS(BLUE_LED_NODE, gpios)

#define HCSR_1 				dist_sens

#define HCSR_1_NODE 		DT_NODELABEL(HCSR_1)

#define HCSR_1_LABEL 		DT_PROP(HCSR_1_NODE, label)

#define HCSR_2 				dist_sens2

#define HCSR_2_NODE 		DT_NODELABEL(HCSR_2)

#define HCSR_2_LABEL 		DT_PROP(HCSR_2_NODE, label)
	

/* Timer declarations*/
K_TIMER_DEFINE(sync_timer, NULL, NULL);

/* CoAP socket fd */
static int sock;

static struct coap_observer observers[NUM_OBSERVERS];

static struct coap_pending pendings[NUM_PENDINGS];

static struct k_work_delayable observer_work;

static int dist_change1, dist_change2, dist_change3, dist_change4;

static struct coap_resource *resource_to_notify;

static struct k_work_delayable retransmit_work;

const static struct device *dev_ledr, *dev_ledg, *dev_ledb, *sensor1, *sensor2;

uint8_t r_status=0, b_status=0, g_status=0 , led_number=0, sensor_number=1;

/* Thread objects creation */
#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

void sensor_measure(void *, void *, void *);


K_THREAD_DEFINE(my_tid, MY_STACK_SIZE,
                sensor_measure, NULL, NULL, NULL,
                MY_PRIORITY, 0, 0);

/* HCSR04 distance measure function */
uint16_t cur1_1=0, cur1_2=0, prev1_1 = 0, prev1_2 =0, cur2_1=0, cur2_2=0, prev2_1=0, prev2_2=0 ;

static int measure(const struct device *dev)
{
    int ret;
    struct sensor_value distance;

    ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
    switch (ret) {
    case 0:
        ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
        if (ret) {
            LOG_ERR("sensor_channel_get failed ret %d", ret);
            return ret;
        }
		if (!strcmp(dev-> name, HCSR_1_LABEL)){
		cur1_1=distance.val1;
		cur1_2=(distance.val2)/1000;
		}
		if (!strcmp(dev-> name, HCSR_2_LABEL)){
		cur2_1=distance.val1;
		cur2_2=(distance.val2)/1000;
		}
        break;
    case -EIO:
        LOG_WRN("%s: Could not read device", dev->name);
        break;
    default:
        LOG_ERR("Error when reading device: %s", dev->name);
        break;
    }
    return 0;
}
/* thread entry point function */
void sensor_measure(void* a, void* b, void* c)
{	
	/* initial timer start */
	k_timer_start(&sync_timer, K_MSEC(1000), K_MSEC(1000));
	
	volatile uint16_t ret=0;
    while (1) {
		k_timer_status_sync(&sync_timer);
		prev1_1=cur1_1;
		prev1_2=cur1_2;
		ret = measure(sensor1);
		if(ret){
			LOG_ERR("Could not measure sensor1");
		}
		prev2_1=cur2_1;
		prev2_2=cur2_2;
		ret = measure(sensor2);
		if(ret){
			LOG_ERR("Could not measure sensor2");
		}

    }
    /* thread terminates at end of entry point function */
}

#define ADDRLEN(sock) \
	(((struct sockaddr *)sock)->sa_family == AF_INET ? \
		sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6))

#if defined(CONFIG_NET_IPV4)
static int start_coap_server(void)
{
	struct sockaddr_in addr;
	int r;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(MY_COAP_PORT);

	sock = socket(addr.sin_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create UDP socket %d", errno);
		return -errno;
	}

	r = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (r < 0) {
		LOG_ERR("Failed to bind UDP socket %d", errno);
		return -errno;
	}

	return 0;
}
#endif

static int send_coap_reply(struct coap_packet *cpkt,
			   const struct sockaddr *addr,
			   socklen_t addr_len)
{
	int r;

	net_hexdump("Response", cpkt->data, cpkt->offset);

	r = sendto(sock, cpkt->data, cpkt->offset, 0, addr, addr_len);
	if (r < 0) {
		LOG_ERR("Failed to send %d", errno);
		r = -errno;
	}

	return r;
}

/* Used for resource discovery */
static int well_known_core_get(struct coap_resource *resource,
			       struct coap_packet *request,
			       struct sockaddr *addr, socklen_t addr_len)
{
	struct coap_packet response;
	uint8_t *data;
	int r;

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_well_known_core_get(resource, request, &response,
				     data, MAX_COAP_MSG_LEN);
	if (r < 0) {
		goto end;
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}
/*
static void schedule_next_retransmission(void)
{
	struct coap_pending *pending;
	int32_t remaining;
	uint32_t now = k_uptime_get_32();

	// Get the first pending retansmission to expire after cycling. 
	pending = coap_pending_next_to_expire(pendings, NUM_PENDINGS);
	if (!pending) {
		return;
	}

	remaining = pending->t0 + pending->timeout - now;
	if (remaining < 0) {
		remaining = 0;
	}

	k_work_reschedule(&retransmit_work, K_MSEC(remaining));
}
*/

//static void remove_observer(struct sockaddr *addr);



/* LED status get method */
static int led_get(struct coap_resource *resource,
		     struct coap_packet *request,
		     struct sockaddr *addr, socklen_t addr_len)

{	
	static struct coap_block_context ctx;
	struct coap_packet response;
	uint8_t payload[64];
	uint8_t token[COAP_TOKEN_MAX_LEN];
	uint8_t *data;
	uint16_t id;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	int r;
	
	if (ctx.total_size == 0) {
		coap_block_transfer_init(&ctx, COAP_BLOCK_64,
					 BLOCK_WISE_TRANSFER_SIZE_GET);
	}

	r = coap_update_from_block(request, &ctx);
	if (r < 0) {
		return -EINVAL;
	}

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, COAP_TYPE_ACK, tkl, token,
			     COAP_RESPONSE_CODE_CONTENT, id);
	if (r < 0) {
		return -EINVAL;
	}

	r = coap_append_option_int(&response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_TEXT_PLAIN);
	if (r < 0) {
		goto end;
	}

	r = coap_append_block2_option(&response, &ctx);
	if (r < 0) {
		goto end;
	}

	r = coap_packet_append_payload_marker(&response);
	if (r < 0) {
		goto end;
	}

led_number = (int)((struct coap_core_metadata *)resource->user_data)->user_data;

	switch(led_number)
	{
		case RED_LED:
			if(r_status == 0)
				snprintk((char *)payload, sizeof(payload), "OFF");
			else
				r = snprintk((char *)payload, sizeof(payload), "ON");
			break;
		case GREEN_LED:
			if(g_status == 0)
				snprintk((char *)payload, sizeof(payload), "OFF");
			else
				r = snprintk((char *)payload, sizeof(payload), "ON");
			break;

		case BLUE_LED:
			if(b_status == 0)
				snprintk((char *)payload, sizeof(payload), "OFF");
			else
				r = snprintk((char *)payload, sizeof(payload), "ON");
			break;
	}

	r = coap_packet_append_payload(&response, (uint8_t*)payload, strlen(payload));
	if (r < 0) {
		goto end;
	}

	r = coap_next_block(&response, &ctx);
	if (!r) {
		/* Will return 0 when it's the last block. */
		memset(&ctx, 0, sizeof(ctx));
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}
/* Update LED status */
static int led_put(struct coap_resource *resource,
		    struct coap_packet *request,
		    struct sockaddr *addr, socklen_t addr_len)
{
	struct coap_packet response;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	const uint8_t *payload;
	uint8_t *data;
	uint16_t payload_len;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	uint16_t id;
	int r,val,ret;

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	payload = coap_packet_get_payload(request, &payload_len);
	if (payload) {
		net_hexdump("PUT Payload", payload, payload_len);
	}
	
	val = atoi(payload);

	led_number = (int)((struct coap_core_metadata *)resource->user_data)->user_data;

	switch(led_number)
	{
		case RED_LED:
		ret = gpio_pin_set(dev_ledr, RED_LED_PIN, val);
		r_status=val;
		if(ret){
			LOG_WRN("gpio not set!\n");
			return -1;
			}
			break;

		case GREEN_LED:
		ret = gpio_pin_set(dev_ledg, GREEN_LED_PIN, val);
		g_status=val;
		if(ret){
			LOG_WRN("gpio not set!\n");
			return -1;
			}
			break;

		case BLUE_LED:
		ret = gpio_pin_set(dev_ledb, BLUE_LED_PIN, val);
		b_status=val;
		if(ret){
			LOG_WRN("gpio not set!\n");
			return -1;
			}
			break;
	}


	if (type == COAP_TYPE_CON) {
		type = COAP_TYPE_ACK;
	} else {
		type = COAP_TYPE_NON_CON;
	}

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, type, tkl, token,
			     COAP_RESPONSE_CODE_CHANGED, id);
	if (r < 0) {
		goto end;
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}

static int send_notification_packet(const struct sockaddr *addr,
                                    socklen_t addr_len,
                                    uint16_t age, uint16_t id,
                                    const uint8_t *token, uint8_t tkl,
                                    bool is_response, struct coap_resource *resource);


static void retransmit_request(struct k_work *work)
{
        struct coap_pending *pending;

        pending = coap_pending_next_to_expire(pendings, NUM_PENDINGS);
        if (!pending) {
                return;
        }

        if (!coap_pending_cycle(pending)) {
                k_free(pending->data);
                coap_pending_clear(pending);
                return;
        }

        k_work_reschedule(&retransmit_work, K_MSEC(pending->timeout));
}


static void update_distance(struct k_work *work)
{
	dist_change1 = abs(cur1_1 - prev1_1);
	dist_change2 = abs(cur1_2 - prev1_2);

	dist_change3 = abs(cur2_1 - prev2_1);
	dist_change4 = abs(cur2_2 - prev2_2);

	if(dist_change1>=1 || dist_change2> 500 || dist_change3 >=1 || dist_change4 >= 500){
		if (resource_to_notify) {
			coap_resource_notify(resource_to_notify);
		}
	}
	k_work_reschedule(&observer_work, K_SECONDS(5));
}

static int create_pending_request(struct coap_packet *response,
				  const struct sockaddr *addr)
{
	struct coap_pending *pending;
	int r;

	pending = coap_pending_next_unused(pendings, NUM_PENDINGS);
	if (!pending) {
		return -ENOMEM;
	}

	r = coap_pending_init(pending, response, addr,
			      COAP_DEFAULT_MAX_RETRANSMIT);
	if (r < 0) {
		return -EINVAL;
	}

	coap_pending_cycle(pending);

	pending = coap_pending_next_to_expire(pendings, NUM_PENDINGS);
	if (!pending) {
		return 0;
	}

	k_work_reschedule(&retransmit_work, K_MSEC(pending->timeout));

	return 0;
}

/* Notifications sent to observers */
static int send_notification_packet(const struct sockaddr *addr,
				    socklen_t addr_len,
				    uint16_t age, uint16_t id,
				    const uint8_t *token, uint8_t tkl,
				    bool is_response, struct coap_resource *resource)
{
	struct coap_packet response;
	char payload[30];
	uint8_t *data;
	uint8_t type;
	int r;

	if (is_response) {
		type = COAP_TYPE_ACK;
	} else {
		type = COAP_TYPE_CON;
	}

	if (!is_response) {
		id = coap_next_id();
	}

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, type, tkl, token,
			     COAP_RESPONSE_CODE_CONTENT, id);
	if (r < 0) {
		goto end;
	}

	if (age >= 2U) {
		r = coap_append_option_int(&response, COAP_OPTION_OBSERVE, age);
		if (r < 0) {
			goto end;
		}
	}

	r = coap_append_option_int(&response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_TEXT_PLAIN);
	if (r < 0) {
		goto end;
	}

	r = coap_packet_append_payload_marker(&response);
	if (r < 0) {
		goto end;
	}

	if(age){
		if(dist_change1 >=1 || dist_change2>500){
			r = snprintk((char *) payload, sizeof(payload),"change1 > 0.5");
			LOG_DBG("change 1 > 0.5");
		}
	else if(dist_change3 >= 1 || dist_change4 > 500){
			r = snprintk((char *) payload, sizeof(payload),"change2 > 0.5");
			LOG_DBG("change 2 > 0.5");
	}
	}
	else {
		sensor_number = (int)((struct coap_core_metadata *)resource->user_data)->user_data;

		if(sensor_number == 1){
			r = snprintk((char *) payload, sizeof(payload), "distance 1: %d.%03din\n", cur1_1, cur1_2);
			LOG_DBG("distance 1 : %d.%03din " , cur1_1, cur1_2 );
		}
	else{
			r = snprintk((char *) payload, sizeof(payload), "distance 2: %d.%03din\n", cur2_1, cur2_2);
			LOG_DBG("distance 2 : %d.%03din " , cur2_1, cur2_2 );
		}
	}

	if (r < 0) {
		goto end;
	}

	r = coap_packet_append_payload(&response, (uint8_t *)payload,
				       strlen(payload));
	if (r < 0) {
		goto end;
	}

	if (type == COAP_TYPE_CON) {
		r = create_pending_request(&response, addr);
		if (r < 0) {
			goto end;
		}
	}

	k_work_reschedule(&observer_work, K_SECONDS(5));

	r = send_coap_reply(&response, addr, addr_len);

	/* On succesfull creation of pending request, do not free memory */
	if (type == COAP_TYPE_CON) {
		return r;
	}

end:
	k_free(data);

	return r;
}

/* Get the distance value sensed by the sensor */
static int hcsr_get(struct coap_resource *resource,
		     struct coap_packet *request,
		     struct sockaddr *addr, socklen_t addr_len)

{
	struct coap_observer *observer;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	uint16_t id;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	bool observe = true;

	if (!coap_request_is_observe(request)) {
		observe = false;
		goto done;
	}

	observer = coap_observer_next_unused(observers, NUM_OBSERVERS);
	if (!observer) {
		return -ENOMEM;
	}

	coap_observer_init(observer, request, addr);

	coap_register_observer(resource, observer);

	resource_to_notify = resource;
	
done:
	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	return send_notification_packet(addr, addr_len,
					observe ? resource->age : 0,
					id, token, tkl, true , resource);
}	

/* Set sample period for HCSR04 */
static int sensor_period_put(struct coap_resource *resource,
		    struct coap_packet *request,
		    struct sockaddr *addr, socklen_t addr_len)
{
	struct coap_packet response;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	const uint8_t *payload;
	uint8_t *data;
	uint16_t payload_len;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	uint16_t id;
	int r,val;

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	payload = coap_packet_get_payload(request, &payload_len);
	if (payload) {
		net_hexdump("PUT Payload", payload, payload_len);
	}
	
	val = atoi(payload);

	//LOG_INF("period value %d", val);

	k_timer_start(&sync_timer, K_MSEC(val), K_MSEC(val));

	if (type == COAP_TYPE_CON) {
		type = COAP_TYPE_ACK;
	} else {
		type = COAP_TYPE_NON_CON;
	}

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, type, tkl, token,
			     COAP_RESPONSE_CODE_CHANGED, id);
	if (r < 0) {
		goto end;
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}

static void hcsr_notify(struct coap_resource *resource,
		       struct coap_observer *observer)
{
	send_notification_packet(&observer->addr,
				 sizeof(observer->addr),
				 resource->age, 0,
				 observer->token, observer->tkl, false, resource);
}

static struct coap_resource *find_resource_by_observer(
		struct coap_resource *resources, struct coap_observer *o)
{
	struct coap_resource *r;

	for (r = resources; r && r->path; r++) {
		sys_snode_t *node;

		SYS_SLIST_FOR_EACH_NODE(&r->observers, node) {
			if (&o->list == node) {
				return r;
			}
		}
	}

	return NULL;
}


static const char * const led_r_path[] = { "led", "led_r", NULL };
static const char * const led_g_path[] = { "led", "led_g", NULL };
static const char * const led_b_path[] = { "led", "led_b", NULL };
static const char * const hcsr_1_path[] = { "sensor", "hcsr_1", NULL };
static const char * const hcsr_2_path[] = { "sensor", "hcsr_2", NULL };
static const char * const sensor_period_path[] = {"sensor", "period", NULL};

static struct coap_resource resources[] = {
	{ .get = well_known_core_get,
	  .path = COAP_WELL_KNOWN_CORE_PATH,
	},
	{ .get = led_get,
	  .put = led_put,
	  .path = led_r_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)0,
        }),
	},
	{ .get = led_get,
	  .put = led_put,
	  .path = led_g_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)1,
        }),
	},
	{ .get = led_get,
	  .put = led_put,
	  .path = led_b_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)2,
        }),
	},
	{ .get = hcsr_get,
	  .path = hcsr_1_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)1,
        }),
		.notify = hcsr_notify,
	},
	{ .get = hcsr_get,
	  .path = hcsr_2_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)2,
        }),
	   .notify = hcsr_notify,
	},
	{ .put = sensor_period_put,
	  .path = sensor_period_path,
	},
};

static void process_coap_request(uint8_t *data, uint16_t data_len,
				 struct sockaddr *client_addr,
				 socklen_t client_addr_len)
{
	struct coap_packet request;
	struct coap_pending *pending;
	struct coap_option options[16] = { 0 };
	uint8_t opt_num = 16U;
	uint8_t type;
	int r;

	r = coap_packet_parse(&request, data, data_len, options, opt_num);
	if (r < 0) {
		LOG_ERR("Invalid data received (%d)\n", r);
		return;
	}

	type = coap_header_get_type(&request);

	pending = coap_pending_received(&request, pendings, NUM_PENDINGS);
	if (!pending) {
		goto not_found;
	}

	/* Clear CoAP pending request */
	if (type == COAP_TYPE_ACK) {
		k_free(pending->data);
		coap_pending_clear(pending);
	}

	return;

not_found:

	if (type == COAP_TYPE_RESET) {
		struct coap_resource *r;
		struct coap_observer *o;

		o = coap_find_observer_by_addr(observers, NUM_OBSERVERS,
					       client_addr);
		if (!o) {
			LOG_ERR("Observer not found\n");
			goto end;
		}

		r = find_resource_by_observer(resources, o);
		if (!r) {
			LOG_ERR("Observer found but Resource not found\n");
			goto end;
		}

		coap_remove_observer(r, o);

		return;
	}

end:
	r = coap_handle_request(&request, resources, options, opt_num,
				client_addr, client_addr_len);
	if (r < 0) {
		LOG_WRN("No handler for such request (%d)\n", r);
	}
}

static int process_client_request(void)
{
	int received;
	struct sockaddr client_addr;
	socklen_t client_addr_len;
	uint8_t request[MAX_COAP_MSG_LEN];

	do {
		client_addr_len = sizeof(client_addr);
		received = recvfrom(sock, request, sizeof(request), 0,
				    &client_addr, &client_addr_len);
		if (received < 0) {
			LOG_ERR("Connection error %d", errno);
			return -errno;
		}

		process_coap_request(request, received, &client_addr,
				     client_addr_len);
	} while (true);

	return 0;
}


void main(void)
{
	int r,ret=0;

	LOG_DBG("Start CoAP-server sample");

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));


	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_GPIO3_IO15, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_GPIO3_IO15,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
			    
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 1);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));	
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
			    
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02, 1);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	 if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT)) {
        /* Give RTT log time to be flushed before executing tests */
        k_sleep(K_MSEC(500));
    }
	/* SENSOR Device configs */
	sensor1= device_get_binding(HCSR_1_LABEL);
	if(sensor1 == NULL)
    {
        LOG_WRN("Failed to get Blue LED Device\n");
        return;
    }
	//LOG_INF("Sensor 1 created");

	sensor2= device_get_binding(HCSR_2_LABEL);
	if(sensor2 == NULL)
    {
        LOG_WRN("Failed to get Blue LED Device\n");
        return;
    }
	//LOG_INF("Sensor 2 created");

	k_thread_start(my_tid);
	/* RED LED Config */
	dev_ledr = device_get_binding(RED_LED_LABEL);
    if(dev_ledr == NULL)
    {
        LOG_WRN("Failed to get Blue LED Device\n");
        return;
    }

	ret = gpio_pin_configure(dev_ledr, RED_LED_PIN, GPIO_OUTPUT_ACTIVE | RED_LED_FLAGS); 
	if(ret < 0)
		LOG_WRN("Error configuring Blue LED pin\n");
	
	ret = gpio_pin_set(dev_ledr, RED_LED_PIN, r_status);
	if(ret){
		LOG_WRN("gpio not set!\n");
		return;
	}

	/* GREEN LED Config */
	dev_ledg = device_get_binding(GREEN_LED_LABEL);
    if(dev_ledg == NULL)
    {
        LOG_WRN("Failed to get Blue LED Device\n");
        return;
    }

	ret = gpio_pin_configure(dev_ledg, GREEN_LED_PIN, GPIO_OUTPUT_ACTIVE | GREEN_LED_FLAGS); 
	if(ret < 0)
		LOG_WRN("Error configuring Blue LED pin\n");
	
	ret = gpio_pin_set(dev_ledg, GREEN_LED_PIN, g_status);
	if(ret){
		LOG_WRN("gpio not set!\n");
		return;
	}

	/* BLUE LED Config */
	dev_ledb = device_get_binding(BLUE_LED_LABEL);
    if(dev_ledb == NULL)
    {
        LOG_WRN("Failed to get Blue LED Device\n");
        return;
    }

	ret = gpio_pin_configure(dev_ledb, BLUE_LED_PIN, GPIO_OUTPUT_ACTIVE | BLUE_LED_FLAGS); 
	if(ret < 0)
		LOG_WRN("Error configuring Blue LED pin\n");
	
	ret = gpio_pin_set(dev_ledb, BLUE_LED_PIN, b_status);
	if(ret){
		LOG_WRN("gpio not set!\n");
		return;
	}


	r = start_coap_server();
	if (r < 0) {
		goto quit;
	}

	k_work_init_delayable(&retransmit_work, retransmit_request);
	k_work_init_delayable(&observer_work, update_distance);

	while (1) {
		r = process_client_request();
		if (r < 0) {
			goto quit;
		}
	}

	LOG_DBG("Done");
	return;

quit:
	LOG_ERR("Quit");
}
