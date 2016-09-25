/*!
 * \file unpack_2bit_iprx_samples.cc
 *
 * \brief Unpacks 2 bit samples that have been packed into bytes or shorts
 * \author Cillian O'Driscoll cillian.odriscoll (at) gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "unpack_2bit_iprx_samples.h"
#include <gnuradio/io_signature.h>
#include <glog/logging.h>

struct byte_2bit_struct_signed
{
  signed sample_0:2;  // <- 2 bits wide only
  signed sample_1:2;  // <- 2 bits wide only
  signed sample_2:2;  // <- 2 bits wide only
  signed sample_3:2;  // <- 2 bits wide only
};

struct byte_2bit_struct
{
  unsigned sample_0:2;  // <- 2 bits wide only
  unsigned sample_1:2;  // <- 2 bits wide only
  unsigned sample_2:2;  // <- 2 bits wide only
  unsigned sample_3:2;  // <- 2 bits wide only
};

const signed int LUT[] = {
    0x01,
    0x03,
    0xFF,
    0xFD
};

union byte_and_samples_signed
{
    int8_t byte;
    byte_2bit_struct_signed samples;
};

union byte_and_samples
{
    uint8_t byte;
    byte_2bit_struct samples;
};

bool systemIsBigEndianIPRX()
{
    union
    {
        uint32_t i;
        char c[4];
    } test_int = {0x01020304};

    return test_int.c[0] == 1; 
}

bool systemBytesAreBigEndianIPRX()
{
    byte_and_samples_signed b;
    b.byte = static_cast< int8_t>(0x1B);

    return b.samples.sample_0 == 0x3;
}

void swapEndiannessIPRX( int8_t const *in, std::vector< int8_t > &out, size_t item_size, unsigned int ninput_items )
{
    unsigned int i;
    unsigned int j = 0;
    int k = 0;
    int l = 0;
    size_t skip = item_size - 1;

    for( i = 0; i < ninput_items; ++i )
    {
        k = j + skip;
        l = j;
        while( k >= l )
        {
            out[j++] = in[k--];
        }
    }
}

unpack_2bit_iprx_samples_sptr make_unpack_2bit_iprx_samples( bool big_endian_bytes,
                                                   size_t item_size,
                                                   bool big_endian_items,
                                                   bool reverse_interleaving )
{
    return unpack_2bit_iprx_samples_sptr(
            new unpack_2bit_iprx_samples( big_endian_bytes,
                                     item_size,
                                     big_endian_items,
                                     reverse_interleaving )
            );
}

unpack_2bit_iprx_samples::unpack_2bit_iprx_samples( bool big_endian_bytes,
                                          size_t item_size,
                                          bool big_endian_items,
                                          bool reverse_interleaving )
    : sync_interpolator("unpack_2bit_samples",
                        gr::io_signature::make(1, 1, item_size),
                        gr::io_signature::make(1, 1, sizeof(char)),
                        4*item_size ), // we make 4 bytes out for every byte in
      big_endian_bytes_(big_endian_bytes),
      item_size_(item_size),
      big_endian_items_(big_endian_items),
      swap_endian_items_(false),
      reverse_interleaving_(reverse_interleaving)
{

    bool big_endian_system = systemIsBigEndianIPRX();

    // Only swap the item bytes if the item size > 1 byte and the system 
    // endianess is not the same as the item endianness:
    swap_endian_items_ = ( item_size_ > 1 ) && 
                         ( big_endian_system != big_endian_items);

    bool big_endian_bytes_system = systemBytesAreBigEndianIPRX();

    swap_endian_bytes_ = ( big_endian_bytes_system != big_endian_bytes_ );

}

unpack_2bit_iprx_samples::~unpack_2bit_iprx_samples()
{}

int unpack_2bit_iprx_samples::work(int noutput_items,
                                   gr_vector_const_void_star &input_items,
                                   gr_vector_void_star &output_items)
{
    signed char const *in = (signed char const *)input_items[0];
    int8_t *out = (int8_t*)output_items[0];

    size_t ninput_bytes = noutput_items/4;
    size_t ninput_items = ninput_bytes/item_size_;
    
    // Handle endian swap if needed
    if( swap_endian_items_ )
    {
        work_buffer_.reserve( ninput_bytes );
        swapEndiannessIPRX( in, work_buffer_, item_size_, ninput_items );

        in = const_cast< signed char const *> ( &work_buffer_[0] );
    }

    // Here the in pointer can be interpreted as a stream of bytes to be
    // converted. But we now have two possibilities:
    // 1) The samples in a byte are in big endian order
    // 2) The samples in a byte are in little endian order

    byte_and_samples raw_byte;
    int n = 0;
    //char dbgbuf[50];

    if( reverse_interleaving_ )
    {
        if( swap_endian_bytes_ )
        {
            for(unsigned int i = 0; i < ninput_bytes; ++i)
            {
                // Read packed input sample (1 byte = 4 samples)
                raw_byte.byte = in[i];

                out[n++] = (int8_t)LUT[raw_byte.samples.sample_3];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_2];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_1];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_0];
		
		/*sprintf(dbgbuf, "%02X", in[i]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: in  " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", (unsigned char)raw_byte.samples.sample_3, (unsigned char)raw_byte.samples.sample_2, (unsigned char)raw_byte.samples.sample_1, (unsigned char)raw_byte.samples.sample_0);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", LUT[raw_byte.samples.sample_3], LUT[raw_byte.samples.sample_2], LUT[raw_byte.samples.sample_1], LUT[raw_byte.samples.sample_0]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;*/

            }
        }
        else
        {
            for(unsigned int i = 0; i < ninput_bytes; ++i )
            {

                // Read packed input sample (1 byte = 4 samples)
                raw_byte.byte = in[i];

                out[n++] = (int8_t)LUT[raw_byte.samples.sample_0];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_1];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_2];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_3];

		/*sprintf(dbgbuf, "%02X", in[i]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: in  " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", (unsigned char)raw_byte.samples.sample_0, (unsigned char)raw_byte.samples.sample_1, (unsigned char)raw_byte.samples.sample_2, (unsigned char)raw_byte.samples.sample_3);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", LUT[raw_byte.samples.sample_0], LUT[raw_byte.samples.sample_1], LUT[raw_byte.samples.sample_2], LUT[raw_byte.samples.sample_3]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;*/
            }
        }
    }
    else
    {

        if( swap_endian_bytes_ )
        {
            for(unsigned int i = 0; i < ninput_bytes; ++i)
            {
                // Read packed input sample (1 byte = 4 samples)
                raw_byte.byte = in[i];

                out[n++] = (int8_t)LUT[raw_byte.samples.sample_2];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_3];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_0];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_1];

		/*sprintf(dbgbuf, "%02X", in[i]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: in  " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", (unsigned char)raw_byte.samples.sample_2, (unsigned char)raw_byte.samples.sample_3, (unsigned char)raw_byte.samples.sample_0, (unsigned char)raw_byte.samples.sample_1);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", LUT[raw_byte.samples.sample_2], LUT[raw_byte.samples.sample_3], LUT[raw_byte.samples.sample_0], LUT[raw_byte.samples.sample_1]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;*/

            }
        }
        else
        {
            for(unsigned int i = 0; i < ninput_bytes; ++i )
            {

                // Read packed input sample (1 byte = 4 samples)
                raw_byte.byte = in[i];

                out[n++] = (int8_t)LUT[raw_byte.samples.sample_1];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_0];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_3];
                out[n++] = (int8_t)LUT[raw_byte.samples.sample_2];

		/*sprintf(dbgbuf, "%02X", in[i]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: in  " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", (unsigned char)raw_byte.samples.sample_1, (unsigned char)raw_byte.samples.sample_0, (unsigned char)raw_byte.samples.sample_3, (unsigned char)raw_byte.samples.sample_2);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;
		sprintf(dbgbuf, "%02X, %02X, %02X, %02X", LUT[raw_byte.samples.sample_1], LUT[raw_byte.samples.sample_0], LUT[raw_byte.samples.sample_3], LUT[raw_byte.samples.sample_2]);
		LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;*/
            }
        }
    }

    //for(unsigned int i = 0; i < ninput_bytes; ++i) {
    //  sprintf(dbgbuf, "%02X", in[i]);
    //  LOG(WARNING) << "unpack_2bit_iprx_samples: in  " << dbgbuf;
    //  sprintf(dbgbuf, "%02X, %02X, %02X, %02X", out[i*4+0], out[i*4+1], out[i*4+2], out[i*4+3]);
    //  LOG(WARNING) << "unpack_2bit_iprx_samples: out " << dbgbuf;
    //}

    return noutput_items;
}

