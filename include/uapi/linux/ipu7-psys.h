/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Copyright (C) 2013 - 2023 Intel Corporation */

#ifndef _UAPI_IPU_PSYS_H
#define _UAPI_IPU_PSYS_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

struct ipu_psys_capability {
	uint32_t version;
	uint8_t driver[20];
	uint8_t dev_model[32];
	uint32_t reserved[17];
} __attribute__ ((packed));

/**
 * PSYS event error to user
 */
enum ipu_psys_event_error {
	IPU_PSYS_EVT_ERROR_NONE = 0U,
	IPU_PSYS_EVT_ERROR_INTERNAL = 1U,
	IPU_PSYS_EVT_ERROR_FRAME = 2U,
	IPU_PSYS_EVT_ERROR_FORCE_CLOSED = 3U,
	IPU_PSYS_EVT_ERROR_MAX
} __attribute__ ((packed));

/**
 * struct ipu_psys_event - event back from driver to user for requested tasks
 * @graph_id:		unique id per graph
 * @node_ctx_id:	unique logical context id per cb
 * @frame_id:		unique id per frame, originally assigned by user
 * @error:		error code of ipu_psys_event_error type
 */
struct ipu_psys_event {
	uint8_t graph_id;
	uint8_t node_ctx_id;
	uint8_t frame_id;
	uint32_t error;
	int32_t reserved[2];
} __attribute__ ((packed));

/**
 * struct ipu_psys_buffer - for input/output terminals
 * @len:	total allocated size @ base address
 * @userptr:	user pointer
 * @fd:		DMA-BUF handle
 * @data_offset:offset to valid data
 * @bytes_used:	amount of valid data including offset
 * @flags:	flags
 */
struct ipu_psys_buffer {
	uint64_t len;
	union {
		int fd;
		void __user *userptr;
		uint64_t reserved;
	} base;
	uint32_t data_offset;
	uint32_t bytes_used;
	uint32_t flags;
	uint32_t reserved[2];
} __attribute__ ((packed));

/**< Max number of logical node */
#define MAX_GRAPH_NODES			5U
/**< Max number of profile */
#define MAX_GRAPH_NODE_PROFILES		1U
#define MAX_GRAPH_LINKS			10U
#define MAX_GRAPH_TERMINALS		32U

/**
 * Settings per node on the bitmap
 * @teb:	Terminal Enable bitmap
 * @deb:	Device Enable bitmap
 * @rbm:	Routing bitmap
 * @reb:	Routing Enable bitmap
 */
struct node_profile {
	uint32_t teb[2];
	uint32_t deb[4];
	uint32_t rbm[4];
	uint32_t reb[4];
} __attribute__ ((packed));

/**
 * struct node_ternimal - terminal description on the node
 *
 * Terminal is the logical connection entity that is in the node,
 * it can be different types, one node could have multiple terminal.
 *
 * @term_id:	id of the terminal
 * @buf_size:	payload(PAC or SDI) size of the certain terminal
 */
struct node_ternimal {
	uint8_t term_id;
	uint32_t buf_size;
} __attribute__ ((packed));

/**
 * struct graph_node - Description of graph that will be used for device
 * and graph open purpose
 *
 * Node is the logical entity of a graph, one graph could have multiple
 * nodes and it could have connection between each node with terminal.
 *
 * @node_rsrc_id:	Physical node id
 * @node_ctx_id:	Logical node id, unique per graph
 * @num_terms:		Number of enabled terms in the node
 * @profiles:		bitmap settings on the node
 * @terminals:		terminal info on the node
 * @num_frags:		Number of fragments
 */
struct graph_node {
	uint8_t node_rsrc_id;
	uint8_t node_ctx_id;
	uint8_t num_terms;
	struct node_profile profiles[MAX_GRAPH_NODE_PROFILES];
	struct node_ternimal terminals[MAX_GRAPH_TERMINALS];
} __attribute__ ((packed));

/**
 * struct graph_link_ep - link endpoint description
 *
 * Link endpoint is used to describe the connection between different nodes.
 *
 * @node_ctx_id:	Node ID as described in the list of nodes in the fgraph
 * @term_id:		Term ID as described in the list of terms in the fgraph
 */
struct graph_link_ep {
	uint8_t node_ctx_id;
	uint8_t term_id;
} __attribute__ ((packed));

/**
 * All local links (links between nodes within a subsystem) require this
 * value to be set.
 */
#define IPU_PSYS_FOREIGN_KEY_NONE UINT16_MAX
/** None value of TOP pbk id if not used */
#define IPU_PSYS_LINK_PBK_ID_NONE UINT8_MAX
/** None value of TOP pbk slot id if not used */
#define IPU_PSYS_LINK_PBK_SLOT_ID_NONE UINT8_MAX
/** Static Offline */
#define IPU_PSYS_LINK_STREAMING_MODE_SOFF 0U

/**
 * struct graph_link - graph link to connect between cbs
 *
 * The sink and source links are defined with terminal information.
 *
 * @ep_src:		Source side of the link
 * @ep_dst:		Destination side of the link
 * @foreign_key:	MUST set to IPU_PSYS_FOREIGN_KEY_NONE
 * @streaming_mode:	Value should be set from IPU_PSYS_LINK_STREAMING_MODE_X
 * @pbk_id:		TOP PBK id that used to connected to external IP
 * @pbk_slot_id:	TOP PBK slot id that used to connected to external IP
 * @delayed_link:	A delay link between producer N and consumer N+1 frame
 */
struct graph_link {
	struct graph_link_ep ep_src;
	struct graph_link_ep ep_dst;
	uint16_t foreign_key;
	uint8_t streaming_mode;
	uint8_t pbk_id;
	uint8_t pbk_slot_id;
	uint8_t delayed_link;
} __attribute__ ((packed));

/**
 * struct ipu_psys_graph_info
 *
 * Topology that describes an IPU internal connection includes CB and terminal
 * information.
 *
 * @graph_id:	id of graph, set initial to 0xFF by user, returned by driver
 * @num_nodes:	number of nodes in graph
 * @nodes:	node entity
 * @links:	link entity
 */
struct ipu_psys_graph_info {
	uint8_t graph_id;
	uint8_t num_nodes;
	struct graph_node __user *nodes;
	struct graph_link links[MAX_GRAPH_LINKS];
} __attribute__ ((packed));

/**
 * struct ipu_psys_term_buffers
 *
 * Descprion of each terminal payload buffer
 *
 * @term_id:	terminal id
 * @term_buf:	terminal buffer
 */
struct ipu_psys_term_buffers {
	uint8_t term_id;
	struct ipu_psys_buffer term_buf;
} __attribute__ ((packed));

/**
 * struct ipu_psys_task_request
 *
 * Task request is for user to send a request associated with terminal
 * payload and expect IPU to process, each task request would expect
 * an event, @see ipu_psys_event
 *
 * @graph_id:		graph id returned from graph open
 * @node_ctx_id:	unique logical context id per cb
 * @frame_id:		frame id
 * @payload_reuse_bm:	Any terminal marked here must be enabled
 * @term_buf_count:	the number of terminal buffers
 * @task_buffers:	terminal buffers on the task request
 * @num_frags:         the number of fragments
 * @frag_buffers:      the buffer information of fragments
 */
struct ipu_psys_task_request {
	uint8_t graph_id;
	uint8_t node_ctx_id;
	uint8_t frame_id;
	uint32_t payload_reuse_bm[2];
	uint8_t term_buf_count;
	struct ipu_psys_term_buffers __user *task_buffers;
} __attribute__ ((packed));

#define IPU_BUFFER_FLAG_INPUT	(1 << 0)
#define IPU_BUFFER_FLAG_OUTPUT	(1 << 1)
#define IPU_BUFFER_FLAG_MAPPED	(1 << 2)
#define IPU_BUFFER_FLAG_NO_FLUSH	(1 << 3)
#define IPU_BUFFER_FLAG_DMA_HANDLE	(1 << 4)
#define IPU_BUFFER_FLAG_USERPTR	(1 << 5)

#define IPU_IOC_QUERYCAP _IOR('A', 1, struct ipu_psys_capability)
#define IPU_IOC_MAPBUF _IOWR('A', 2, int)
#define IPU_IOC_UNMAPBUF _IOWR('A', 3, int)
#define IPU_IOC_GETBUF _IOWR('A', 4, struct ipu_psys_buffer)
#define IPU_IOC_PUTBUF _IOWR('A', 5, struct ipu_psys_buffer)
#define IPU_IOC_DQEVENT _IOWR('A', 6, struct ipu_psys_event)
#define IPU_IOC_GRAPH_OPEN _IOWR('A', 7, struct ipu_psys_graph_info)
#define IPU_IOC_TASK_REQUEST _IOWR('A', 8, struct ipu_psys_task_request)
#define IPU_IOC_GRAPH_CLOSE _IOWR('A', 9, int)

#endif /* _UAPI_IPU_PSYS_H */
