// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmtypes/kinect_link_msg_t.h"

static int __kinect_link_msg_t_hash_computed;
static uint64_t __kinect_link_msg_t_hash;

uint64_t __kinect_link_msg_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __kinect_link_msg_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__kinect_link_msg_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0xe071115012961010LL
         + __int32_t_hash_recursive(&cp)
         + __kinect_point3d_t_hash_recursive(&cp)
         + __kinect_point3d_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __kinect_link_msg_t_get_hash(void)
{
    if (!__kinect_link_msg_t_hash_computed) {
        __kinect_link_msg_t_hash = (int64_t)__kinect_link_msg_t_hash_recursive(NULL);
        __kinect_link_msg_t_hash_computed = 1;
    }

    return __kinect_link_msg_t_hash;
}

int __kinect_link_msg_t_encode_array(void *buf, int offset, int maxlen, const kinect_link_msg_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].joint_id), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __kinect_point3d_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].source), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __kinect_point3d_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].dest), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int kinect_link_msg_t_encode(void *buf, int offset, int maxlen, const kinect_link_msg_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __kinect_link_msg_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __kinect_link_msg_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __kinect_link_msg_t_encoded_array_size(const kinect_link_msg_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int32_t_encoded_array_size(&(p[element].joint_id), 1);

        size += __kinect_point3d_t_encoded_array_size(&(p[element].source), 1);

        size += __kinect_point3d_t_encoded_array_size(&(p[element].dest), 1);

    }
    return size;
}

int kinect_link_msg_t_encoded_size(const kinect_link_msg_t *p)
{
    return 8 + __kinect_link_msg_t_encoded_array_size(p, 1);
}

int __kinect_link_msg_t_decode_array(const void *buf, int offset, int maxlen, kinect_link_msg_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].joint_id), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __kinect_point3d_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].source), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __kinect_point3d_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].dest), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __kinect_link_msg_t_decode_array_cleanup(kinect_link_msg_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_decode_array_cleanup(&(p[element].joint_id), 1);

        __kinect_point3d_t_decode_array_cleanup(&(p[element].source), 1);

        __kinect_point3d_t_decode_array_cleanup(&(p[element].dest), 1);

    }
    return 0;
}

int kinect_link_msg_t_decode(const void *buf, int offset, int maxlen, kinect_link_msg_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __kinect_link_msg_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __kinect_link_msg_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int kinect_link_msg_t_decode_cleanup(kinect_link_msg_t *p)
{
    return __kinect_link_msg_t_decode_array_cleanup(p, 1);
}

int __kinect_link_msg_t_clone_array(const kinect_link_msg_t *p, kinect_link_msg_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_clone_array(&(p[element].joint_id), &(q[element].joint_id), 1);

        __kinect_point3d_t_clone_array(&(p[element].source), &(q[element].source), 1);

        __kinect_point3d_t_clone_array(&(p[element].dest), &(q[element].dest), 1);

    }
    return 0;
}

kinect_link_msg_t *kinect_link_msg_t_copy(const kinect_link_msg_t *p)
{
    kinect_link_msg_t *q = (kinect_link_msg_t*) malloc(sizeof(kinect_link_msg_t));
    __kinect_link_msg_t_clone_array(p, q, 1);
    return q;
}

void kinect_link_msg_t_destroy(kinect_link_msg_t *p)
{
    __kinect_link_msg_t_decode_array_cleanup(p, 1);
    free(p);
}

int kinect_link_msg_t_publish(lcm_t *lc, const char *channel, const kinect_link_msg_t *p)
{
      int max_data_size = kinect_link_msg_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = kinect_link_msg_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _kinect_link_msg_t_subscription_t {
    kinect_link_msg_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void kinect_link_msg_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    kinect_link_msg_t p;
    memset(&p, 0, sizeof(kinect_link_msg_t));
    status = kinect_link_msg_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding kinect_link_msg_t!!!\n", status);
        return;
    }

    kinect_link_msg_t_subscription_t *h = (kinect_link_msg_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    kinect_link_msg_t_decode_cleanup (&p);
}

kinect_link_msg_t_subscription_t* kinect_link_msg_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    kinect_link_msg_t_handler_t f, void *userdata)
{
    kinect_link_msg_t_subscription_t *n = (kinect_link_msg_t_subscription_t*)
                       malloc(sizeof(kinect_link_msg_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 kinect_link_msg_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg kinect_link_msg_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int kinect_link_msg_t_subscription_set_queue_capacity (kinect_link_msg_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int kinect_link_msg_t_unsubscribe(lcm_t *lcm, kinect_link_msg_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe kinect_link_msg_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

