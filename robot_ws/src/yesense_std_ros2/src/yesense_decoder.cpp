#include <iostream>
#include <string.h>
#include "yesense_decoder_comm.h"
#include "yesense_decoder.h"

// ===========================================================================


// ===========================================================================
namespace yesense{

yesense_decoder::yesense_decoder()
{
    memset(decode_data, 0, DATA_BUF_SIZE);
    decode_buf_len = 0u;
    std_out_decoder = new yis_std_out_decoder();

    cmd_info.flg        = 0u;
    cmd_info.result     = 0x00u;
    cmd_info.data_class = 0x00u;
}

yesense_decoder::~yesense_decoder()
{
    delete std_out_decoder;
}

int yesense_decoder::data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result)
{   
    if(NULL == result || (0u != len && NULL == data))
    {
        return para_err;
    }
    if(len > DATA_BUF_SIZE)
    {
        decode_buf_len = 0u;
        return buf_full;
    }
    if(len > DATA_BUF_SIZE - decode_buf_len)
    {
        // Discard an overflowing backlog so fresh input can resynchronize.
        decode_buf_len = 0u;
    }
    if(0u != len)
    {
        memcpy(decode_data + decode_buf_len, data, len);
        decode_buf_len += len;
    }

    while(0u != decode_buf_len)
    {
        const int ret = std_out_decoder->data_proc(decode_data, decode_buf_len, result);
        msg_idx_t *msg_idx = std_out_decoder->msg_idx_obt();
        if(analysis_ok == ret)
        {
            // Remove both leading noise and the successfully decoded packet.
            clear_buf_data(0u, msg_idx->end_idx);
            return ret;
        }
        if(no_header == ret)
        {
            // A header may be split between two serial reads (59 | 53).
            const bool keep_header = decode_data[decode_buf_len - 1u] == 0x59u;
            decode_buf_len = keep_header ? 1u : 0u;
            if(keep_header) decode_data[0] = 0x59u;
            return ret;
        }
        if(data_len_err == ret)
        {
            // Keep the incomplete packet, but discard preceding noise.
            if(0u != msg_idx->st_idx)
                clear_buf_data(0u, msg_idx->st_idx - 1u);
            return ret;
        }
        if(crc_err == ret || para_err == ret)
        {
            // Skip the failed header and search again. Never use a stale end index.
            clear_buf_data(0u, msg_idx->st_idx);
            continue;
        }
        return ret;
    }
    return no_header;
}

int yesense_decoder::clear_buf_data(unsigned int st_idx, unsigned int end_idx)
{
    unsigned int cnt = 0u;

    if(st_idx > end_idx || end_idx >= decode_buf_len)
    {
        return para_err;
    }

    cnt = end_idx - st_idx + 1u;
    const unsigned int remaining = decode_buf_len - end_idx - 1u;
    memmove(decode_data + st_idx, decode_data + end_idx + 1u, remaining);
    decode_buf_len -= cnt;

    return analysis_ok;
}

}
