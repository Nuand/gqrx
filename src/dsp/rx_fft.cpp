/* -*- c++ -*- */
/*
 * Gqrx SDR: Software defined radio receiver powered by GNU Radio and Qt
 *           http://gqrx.dk/
 *
 * Copyright 2011-2013 Alexandru Csete OZ9AEC.
 *
 * Gqrx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Gqrx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include <math.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/block.h>
#include <gnuradio/tags.h>
#include "dsp/rx_fft.h"


rx_fft_c_sptr make_rx_fft_c (unsigned int fftsize, int wintype)
{
    return gnuradio::get_initial_sptr(new rx_fft_c (fftsize, wintype));
}

/*! \brief Create receiver FFT object.
 *  \param fftsize The FFT size.
 *  \param wintype The window type (see gr::filter::firdes::win_type).
 *
 */
rx_fft_c::rx_fft_c(unsigned int fftsize, int wintype)
    : gr::sync_block ("rx_fft_c",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(0, 0, 0)),
      d_fftsize(fftsize),
      d_wintype(-1)
{

    /* create FFT object */
    d_fft = new gr::fft::fft_complex(d_fftsize, true);

    /* allocate circular buffer */
    d_cbuf.set_capacity(d_fftsize);

    for (int i = 0; i < 10; i++) {
        d_mult_cbuf[i].set_capacity(d_fftsize);
    }

    d_mult_idx = 0;
    d_mult_size = 3;

    d_mult = true;

    /* create FFT window */
    set_window_type(wintype);
}

rx_fft_c::~rx_fft_c()
{
    delete d_fft;
}

/*! \brief Set sweeping spectrum analyzer mode settings
 *  \param enable Enable sweeping mode
 *  \param num_bin Number of sweeping mode bins
 *  \param halfband Fraction of FFT size that is usable bandwidth
 *  \return 0 on success
 */
int rx_fft_c::set_sweeping_mode(bool enable, int num_bin, double halfband_coeff)
{
    d_mult = enable;
    d_mult_size = num_bin;
    return 0;
}

/*! \brief Receiver FFT work method.
 *  \param noutput_items
 *  \param input_items
 *  \param output_items
 *
 * This method does nothing except throwing the incoming samples into the
 * circular buffer.
 * FFT is only executed when the GUI asks for new FFT data via get_fft_data().
 */
#include <stdio.h>
int rx_fft_c::work(int noutput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items)
{
    int i;
    const gr_complex *in = (const gr_complex*)input_items[0];
    (void) output_items;
    std::vector< gr::tag_t > tags;
    get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + noutput_items);
    std::vector< gr::tag_t >::iterator it = tags.begin();

    //d_mult_idx = 0;
    /* just throw new samples into the buffer */
    boost::mutex::scoped_lock lock(d_mutex);
    for (i = 0; i < noutput_items; i++)
    {
        if (d_mult) {
            if (tags.size()) {
                if (it->offset == (nitems_read(0) + i)) {
                    d_mult_idx = to_long(it->value);
                    if (it != tags.end())
                        it++;
//                    std::cout << "FUCK " << d_mult_idx << std::endl;
                }
            }
            d_mult_cbuf[d_mult_idx].push_back(in[i]);
        } else {
            d_cbuf.push_back(in[i]);
        }
    }

    return noutput_items;

}

/*! \brief Get FFT data.
 *  \param fftPoints Buffer to copy FFT data
 *  \param fftSize Current FFT size (output).
 */
void rx_fft_c::get_fft_data(std::complex<float>* fftPoints, unsigned int &fftSize, unsigned int bin)
{
    boost::mutex::scoped_lock lock(d_mutex);

    if (!d_mult) {
        if (d_cbuf.size() < d_fftsize)
        {
            // not enough samples in the buffer
            fftSize = 0;

            return;
        }

        int halfband = (d_fftsize * 14) / 40;
        /* perform FFT */
        do_fft(d_cbuf.linearize(), d_cbuf.size());  // FIXME: array_one() and two() may be faster
        //d_cbuf.clear();

        /* get FFT data */
        memcpy(fftPoints, d_fft->get_outbuf(), sizeof(gr_complex)*d_fftsize);
        fftSize = d_fftsize;
    } else {
        int halfband = (d_fftsize * 14) / 40;
        fftSize = d_fftsize + halfband * ( d_mult_size - 1 );

        for (int i = 0; i < d_fftsize * 4; i++) {
            fftPoints[i] = 0;
        }
        for (int i = 0; i < d_mult_size; i++) {
            if (d_mult_cbuf[i].size() < d_fftsize)
            {
                // not enough samples in the buffer
                continue;
            }

            /* perform FFT */
            do_fft(d_mult_cbuf[i].linearize(), d_mult_cbuf[i].size());  // FIXME: array_one() and two() may be faster
            //d_cbuf.clear();

            if( i == 0 ) {
                /* get FFT data */
                memcpy(fftPoints,                   d_fft->get_outbuf(),                    sizeof(gr_complex)*halfband);
                memcpy(fftPoints+fftSize-halfband,  d_fft->get_outbuf()+d_fftsize-halfband, sizeof(gr_complex)*halfband);
            } else {
                if ( i & 1 ) {
                    // odd = left
                    if ((i+2) == d_mult_size) {
                        //memcpy(fftPoints+fftSize -halfband*((i+1)/2)-d_fftsize/2, d_fft->get_outbuf()+d_fftsize/2, sizeof(gr_complex)*d_fftsize/2);
                        memcpy(fftPoints+fftSize -halfband*((i+1)/2)-d_fftsize/2, d_fft->get_outbuf()+d_fftsize/2, sizeof(gr_complex)*(d_fftsize-3)/2);
                    } else {
                        memcpy(fftPoints+fftSize -halfband*((i+1)/2+1), d_fft->get_outbuf()+d_fftsize-halfband, sizeof(gr_complex)*halfband);
                    }
                } else {
                    // even = right
                    if ((i+1) == d_mult_size) {
                        //memcpy(fftPoints         +halfband*(i/2), d_fft->get_outbuf(), sizeof(gr_complex)*d_fftsize/2);
                        memcpy(fftPoints         +halfband*(i/2)+3, d_fft->get_outbuf()+3, sizeof(gr_complex)*(d_fftsize/2 - 3));
                    } else {
                        memcpy(fftPoints         +halfband*(i/2), d_fft->get_outbuf(), sizeof(gr_complex)*halfband);
                    }
                }
            }
        }

    }
}

/*! \brief Compute FFT on the available input data.
 *  \param data_in The data to compute FFT on.
 *  \param size The size of data_in.
 *
 * Note that this function does not lock the mutex since the caller, get_fft_data()
 * has alrady locked it.
 */
void rx_fft_c::do_fft(const gr_complex *data_in, unsigned int size)
{
    /* apply window, if any */
    if (d_window.size())
    {
        gr_complex *dst = d_fft->get_inbuf();
        for (unsigned int i = 0; i < size; i++)
            dst[i] = data_in[i] * d_window[i];
    }
    else
    {
        memcpy(d_fft->get_inbuf(), data_in, sizeof(gr_complex)*size);
    }

    /* compute FFT */
    d_fft->execute();
}

/*! \brief Set new FFT size. */
void rx_fft_c::set_fft_size(unsigned int fftsize)
{
    if (fftsize != d_fftsize)
    {
        boost::mutex::scoped_lock lock(d_mutex);

        d_fftsize = fftsize;

        /* clear and resize circular buffer */
        d_cbuf.clear();
        d_cbuf.set_capacity(d_fftsize);
        for (int i = 0 ; i < 10 ; i++) {
            d_mult_cbuf[i].clear();
            d_mult_cbuf[i].set_capacity(d_fftsize);
        }

        /* reset window */
        int wintype = d_wintype; // FIXME: would be nicer with a window_reset()
        d_wintype = -1;
        set_window_type(wintype);

        /* reset FFT object (also reset FFTW plan) */
        delete d_fft;
        d_fft = new gr::fft::fft_complex (d_fftsize, true);
    }

}

/*! \brief Get currently used FFT size. */
unsigned int rx_fft_c::get_fft_size() const
{
    return d_fftsize;
}

/*! \brief Set new window type. */
void rx_fft_c::set_window_type(int wintype)
{
    if (wintype == d_wintype)
    {
        /* nothing to do */
        return;
    }

    d_wintype = wintype;

    if ((d_wintype < gr::filter::firdes::WIN_HAMMING) || (d_wintype > gr::filter::firdes::WIN_FLATTOP))
    {
        d_wintype = gr::filter::firdes::WIN_HAMMING;
    }

    d_window.clear();
    d_window = gr::filter::firdes::window((gr::filter::firdes::win_type)d_wintype, d_fftsize, 6.76);
}

/*! \brief Get currently used window type. */
int rx_fft_c::get_window_type() const
{
    return d_wintype;
}


/**   rx_fft_f     **/

rx_fft_f_sptr make_rx_fft_f(unsigned int fftsize, int wintype)
{
    return gnuradio::get_initial_sptr(new rx_fft_f (fftsize, wintype));
}

/*! \brief Create receiver FFT object.
 *  \param fftsize The FFT size.
 *  \param wintype The window type (see gr::filter::firdes::win_type).
 *
 */
rx_fft_f::rx_fft_f(unsigned int fftsize, int wintype)
    : gr::sync_block ("rx_fft_f",
          gr::io_signature::make(1, 1, sizeof(float)),
          gr::io_signature::make(0, 0, 0)),
      d_fftsize(fftsize),
      d_wintype(-1)
{

    /* create FFT object */
    d_fft = new gr::fft::fft_complex(d_fftsize, true);

    /* allocate circular buffer */
    d_cbuf.set_capacity(d_fftsize);

    /* create FFT window */
    set_window_type(wintype);
}

rx_fft_f::~rx_fft_f()
{
    delete d_fft;
}

/*! \brief Audio FFT work method.
 *  \param noutput_items
 *  \param input_items
 *  \param output_items
 *
 * This method does nothing except throwing the incoming samples into the
 * circular buffer.
 * FFT is only executed when the GUI asks for new FFT data via get_fft_data().
 */
int rx_fft_f::work(int noutput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items)
{
    int i;
    const float *in = (const float*)input_items[0];
    (void) output_items;

    /* just throw new samples into the buffer */
    boost::mutex::scoped_lock lock(d_mutex);
    for (i = 0; i < noutput_items; i++)
    {
        d_cbuf.push_back(in[i]);
    }

    return noutput_items;
}

/*! \brief Get FFT data.
 *  \param fftPoints Buffer to copy FFT data
 *  \param fftSize Current FFT size (output).
 */
void rx_fft_f::get_fft_data(std::complex<float>* fftPoints, unsigned int &fftSize)
{
    boost::mutex::scoped_lock lock(d_mutex);

    if (d_cbuf.size() < d_fftsize)
    {
        // not enough samples in the buffer
        fftSize = 0;

        return;
    }

    /* perform FFT */
    do_fft(d_cbuf.linearize(), d_cbuf.size());  // FIXME: array_one() and two() may be faster
    //d_cbuf.clear();

    /* get FFT data */
    memcpy(fftPoints, d_fft->get_outbuf(), sizeof(gr_complex)*d_fftsize);
    fftSize = d_fftsize;
}

/*! \brief Compute FFT on the available input data.
 *  \param data_in The data to compute FFT on.
 *  \param size The size of data_in.
 *
 * Note that this function does not lock the mutex since the caller, get_fft_data()
 * has alrady locked it.
 */
void rx_fft_f::do_fft(const float *data_in, unsigned int size)
{
    gr_complex *dst = d_fft->get_inbuf();
    unsigned int i;

    /* apply window, and convert to complex */
    if (d_window.size())
    {
        for (i = 0; i < size; i++)
            dst[i] = data_in[i] * d_window[i];
    }
    else
    {
        for (i = 0; i < size; i++)
            dst[i] = data_in[i];
    }

    /* compute FFT */
    d_fft->execute();
}


/*! \brief Set new FFT size. */
void rx_fft_f::set_fft_size(unsigned int fftsize)
{
    if (fftsize != d_fftsize)
    {
        boost::mutex::scoped_lock lock(d_mutex);

        d_fftsize = fftsize;

        /* clear and resize circular buffer */
        d_cbuf.clear();
        d_cbuf.set_capacity(d_fftsize);

        /* reset window */
        int wintype = d_wintype; // FIXME: would be nicer with a window_reset()
        d_wintype = -1;
        set_window_type(wintype);

        /* reset FFT object (also reset FFTW plan) */
        delete d_fft;
        d_fft = new gr::fft::fft_complex(d_fftsize, true);
    }
}

/*! \brief Get currently used FFT size. */
unsigned int rx_fft_f::get_fft_size() const
{
    return d_fftsize;
}

/*! \brief Set new window type. */
void rx_fft_f::set_window_type(int wintype)
{
    if (wintype == d_wintype)
    {
        /* nothing to do */
        return;
    }

    d_wintype = wintype;

    if ((d_wintype < gr::filter::firdes::WIN_HAMMING) || (d_wintype > gr::filter::firdes::WIN_FLATTOP))
    {
        d_wintype = gr::filter::firdes::WIN_HAMMING;
    }

    d_window.clear();
    d_window = gr::filter::firdes::window((gr::filter::firdes::win_type)d_wintype, d_fftsize, 6.76);
}

/*! \brief Get currently used window type. */
int rx_fft_f::get_window_type() const
{
    return d_wintype;
}
