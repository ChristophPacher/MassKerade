/*!
 * @file	blob_detector.cpp
 * @author  Christoph Pacher <chris@christophpacher.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2014 Christoph Pacher http://www.christophpacher.com
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 * the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and
 *   the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *   the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "blob_detector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#define BD_SHOW_DEBUG_IMG

using namespace cv;
using namespace std;
using namespace pacher;
namespace bg = boost::geometry;

#ifdef PC_PERF_MARK
using namespace concurrency::diagnostic;
#endif // PC_PERF_MARK


blob_detector::blob_detector(const blob_detector::settings &s) :
settings_(s),
kernel_shape_(getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1)))
{
#ifdef PC_PERF_MARK
	marker_series_ = make_shared<marker_series>( L"Blob Detector" );
#endif
#ifdef BD_SHOW_DEBUG_IMG
	namedWindow("Blob Detector", WINDOW_AUTOSIZE);
#endif // HD_SHOW_DEBUG_IMG
}

pacher::blobs blob_detector::detect( cv::Mat &in, cv::Mat &index_mask_out )
{
#ifdef PC_PERF_MARK
	span repro_span{ *marker_series_, L"detect Blobs" };
#endif // PC_PERF_MAR

	assert( in.type() == CV_8UC1 );
	assert( index_mask_out.type() == CV_8UC1 );

	Mat in_clone = in.clone();

	morphologyEx(in_clone, in_clone, MORPH_CLOSE, kernel_shape_, Point(-1, -1), settings_.close_num);

	#ifdef BD_SHOW_DEBUG_IMG
	imshow("Blob Detector", in_clone);
	#endif // BD_SHOW_DEBUG_IMG

	vector<contour> c;
	findContours(in_clone, c, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<blob> found_blobs;
	
	for (size_t i = 0; i < c.size(); ++i) {

		bg::correct(c[i]);
		float area = bg::area(c[i]);

		if (area > settings_.min_size) {

			int index = found_blobs.size() + 1;

			found_blobs.emplace_back( move(c[i]), in_clone.cols, in_clone.rows, area );

			// copy to pixels of the blob mask into the index mask image
			// when they are lager than the pixels already in. 
			// in case two mask overlap (no add because of that)
			cv::max( index_mask_out, found_blobs[ index - 1 ].mask_ * index, index_mask_out );
		}
	}
	
	return found_blobs;

}
