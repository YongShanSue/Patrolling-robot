// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifndef _kinect_image_msg_t_h
#define _kinect_image_msg_t_h

#ifdef __cplusplus
extern "C" {
#endif

#define KINECT_IMAGE_MSG_T_VIDEO_RGB 0
#define KINECT_IMAGE_MSG_T_VIDEO_BAYER 1
#define KINECT_IMAGE_MSG_T_VIDEO_IR_8BIT 2
#define KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT 3
#define KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT_PACKED 4
#define KINECT_IMAGE_MSG_T_VIDEO_YUV_RGB 5
#define KINECT_IMAGE_MSG_T_VIDEO_YUV_RAW 6
#define KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG 100
#define KINECT_IMAGE_MSG_T_VIDEO_NONE 101

typedef struct _kinect_image_msg_t kinect_image_msg_t;
struct _kinect_image_msg_t
{
    int64_t    timestamp;
    int16_t    width;
    int16_t    height;
    int32_t    image_data_nbytes;
    uint8_t    *image_data;
    int8_t     image_data_format;
};

/**
 * Create a deep copy of a kinect_image_msg_t.
 * When no longer needed, destroy it with kinect_image_msg_t_destroy()
 */
kinect_image_msg_t* kinect_image_msg_t_copy(const kinect_image_msg_t* to_copy);

/**
 * Destroy an instance of kinect_image_msg_t created by kinect_image_msg_t_copy()
 */
void kinect_image_msg_t_destroy(kinect_image_msg_t* to_destroy);

/**
 * Identifies a single subscription.  This is an opaque data type.
 */
typedef struct _kinect_image_msg_t_subscription_t kinect_image_msg_t_subscription_t;

/**
 * Prototype for a callback function invoked when a message of type
 * kinect_image_msg_t is received.
 */
typedef void(*kinect_image_msg_t_handler_t)(const lcm_recv_buf_t *rbuf,
             const char *channel, const kinect_image_msg_t *msg, void *userdata);

/**
 * Publish a message of type kinect_image_msg_t using LCM.
 *
 * @param lcm The LCM instance to publish with.
 * @param channel The channel to publish on.
 * @param msg The message to publish.
 * @return 0 on success, <0 on error.  Success means LCM has transferred
 * responsibility of the message data to the OS.
 */
int kinect_image_msg_t_publish(lcm_t *lcm, const char *channel, const kinect_image_msg_t *msg);

/**
 * Subscribe to messages of type kinect_image_msg_t using LCM.
 *
 * @param lcm The LCM instance to subscribe with.
 * @param channel The channel to subscribe to.
 * @param handler The callback function invoked by LCM when a message is received.
 *                This function is invoked by LCM during calls to lcm_handle() and
 *                lcm_handle_timeout().
 * @param userdata An opaque pointer passed to @p handler when it is invoked.
 * @return 0 on success, <0 if an error occured
 */
kinect_image_msg_t_subscription_t* kinect_image_msg_t_subscribe(lcm_t *lcm, const char *channel, kinect_image_msg_t_handler_t handler, void *userdata);

/**
 * Removes and destroys a subscription created by kinect_image_msg_t_subscribe()
 */
int kinect_image_msg_t_unsubscribe(lcm_t *lcm, kinect_image_msg_t_subscription_t* hid);

/**
 * Sets the queue capacity for a subscription.
 * Some LCM providers (e.g., the default multicast provider) are implemented
 * using a background receive thread that constantly revceives messages from
 * the network.  As these messages are received, they are buffered on
 * per-subscription queues until dispatched by lcm_handle().  This function
 * how many messages are queued before dropping messages.
 *
 * @param subs the subscription to modify.
 * @param num_messages The maximum number of messages to queue
 *  on the subscription.
 * @return 0 on success, <0 if an error occured
 */
int kinect_image_msg_t_subscription_set_queue_capacity(kinect_image_msg_t_subscription_t* subs,
                              int num_messages);

/**
 * Encode a message of type kinect_image_msg_t into binary form.
 *
 * @param buf The output buffer.
 * @param offset Encoding starts at this byte offset into @p buf.
 * @param maxlen Maximum number of bytes to write.  This should generally
 *               be equal to kinect_image_msg_t_encoded_size().
 * @param msg The message to encode.
 * @return The number of bytes encoded, or <0 if an error occured.
 */
int kinect_image_msg_t_encode(void *buf, int offset, int maxlen, const kinect_image_msg_t *p);

/**
 * Decode a message of type kinect_image_msg_t from binary form.
 * When decoding messages containing strings or variable-length arrays, this
 * function may allocate memory.  When finished with the decoded message,
 * release allocated resources with kinect_image_msg_t_decode_cleanup().
 *
 * @param buf The buffer containing the encoded message
 * @param offset The byte offset into @p buf where the encoded message starts.
 * @param maxlen The maximum number of bytes to read while decoding.
 * @param msg Output parameter where the decoded message is stored
 * @return The number of bytes decoded, or <0 if an error occured.
 */
int kinect_image_msg_t_decode(const void *buf, int offset, int maxlen, kinect_image_msg_t *msg);

/**
 * Release resources allocated by kinect_image_msg_t_decode()
 * @return 0
 */
int kinect_image_msg_t_decode_cleanup(kinect_image_msg_t *p);

/**
 * Check how many bytes are required to encode a message of type kinect_image_msg_t
 */
int kinect_image_msg_t_encoded_size(const kinect_image_msg_t *p);

// LCM support functions. Users should not call these
int64_t __kinect_image_msg_t_get_hash(void);
uint64_t __kinect_image_msg_t_hash_recursive(const __lcm_hash_ptr *p);
int     __kinect_image_msg_t_encode_array(void *buf, int offset, int maxlen, const kinect_image_msg_t *p, int elements);
int     __kinect_image_msg_t_decode_array(const void *buf, int offset, int maxlen, kinect_image_msg_t *p, int elements);
int     __kinect_image_msg_t_decode_array_cleanup(kinect_image_msg_t *p, int elements);
int     __kinect_image_msg_t_encoded_array_size(const kinect_image_msg_t *p, int elements);
int     __kinect_image_msg_t_clone_array(const kinect_image_msg_t *p, kinect_image_msg_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
