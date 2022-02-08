/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 ********************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "Geometry.h"
#include "TagDetector.h"
#include "Refine.h"

enum {
  test_img_w = 30,
  test_img_h = 30,
  test_img_tagsz = 12,
  test_img_scl = 8
};

cv::Mat shrink(const cv::Mat& image, int scl) {
  cv::Mat small;
  cv::resize(image, small, cv::Size(image.cols/scl, image.rows/scl), 0, 0,
             CV_INTER_AREA);
  return small;
}

cv::Mat addNoise(const cv::Mat& src, cv::RNG& rng, double stddev) {

  cv::Mat input;

  if (src.depth() != at::REAL_IMAGE_TYPE) {
    src.convertTo(input, CV_MAKETYPE(at::REAL_IMAGE_TYPE, src.channels()));
  } else {
    input = src.clone();
  }

  std::vector<cv::Mat> chans;

  cv::split(input, chans);

  for (size_t c=0; c<chans.size(); ++c) {
    at::Mat m = chans[c];
    for (int y=0; y<m.rows; ++y) {
      for (int x=0; x<m.cols; ++x) {
        m(y,x) += rng.gaussian(stddev);
      }
    }
  }

  cv::merge(chans, input);

  cv::Mat output; 

  if (input.depth() != src.depth()) {
    input.convertTo(output, CV_MAKETYPE(src.depth(), src.channels()));
  } else {
    output = input;
  }

  return output;

}

size_t fakeDetectionImage(const TagFamily& tagFamily,
                          cv::RNG& rng,
                          at::Point p[4],
                          cv::Mat& small) {

  const int dx[4] = { -1, 1,  1, -1 };
  const int dy[4] = {  1, 1, -1, -1 };
  
  at::real theta = rng.uniform(-1.0,1.0) * M_PI / 4;
  at::real ct = cos(theta);
  at::real st = sin(theta);
  at::Point b1(ct, st);
  at::Point b2(-st, ct);

  for (int i=0; i<4; ++i) {
    p[i] = at::Point(dx[i]*test_img_tagsz*0.5, dy[i]*test_img_tagsz*0.5);
    p[i] = at::Point(b1.dot(p[i]), b2.dot(p[i]));
    p[i].x += test_img_tagsz*rng.uniform(-1.0, 1.0) / 10 + test_img_w/2;
    p[i].y += test_img_tagsz*rng.uniform(-1.0, 1.0) / 10 + test_img_h/2;
    p[i] *= test_img_scl;
  }
  
  int id = rng.uniform(0, int(tagFamily.codes.size()-1));

  at::Point opticalCenter(0.5*test_img_w*test_img_scl, 0.5*test_img_h*test_img_scl);
  
  Quad quad(p, opticalCenter, 4*test_img_scl*test_img_tagsz);

  TagDetection d;
  tagFamily.decode(d, tagFamily.codes[id]);

  for (int i = 0; i < 4; i++) {
    d.p[i] = quad.p[i] * (1.0/test_img_scl);
  }
  
  // compute the homography (and rotate it appropriately)
  d.homography = quad.H;
  d.hxy = quad.opticalCenter;

  at::real c = cos(d.rotation*M_PI/2.0);
  at::real s = sin(d.rotation*M_PI/2.0);
  at::real R[9] = { 
    c, -s, 0, 
    s,  c, 0, 
    0,  0, 1 
  };
  at::Mat Rmat(3, 3, R);
  d.homography = d.homography * Rmat;
  
  d.cxy = quad.interpolate01(.5, .5);
  d.observedPerimeter = quad.observedPerimeter;

  cv::Mat big = tagFamily.detectionImage(d, 
                                         cv::Size(test_img_scl*test_img_w, 
                                                  test_img_scl*test_img_h), 
                                         CV_8UC3, CV_RGB(255,255,255));
  
  small = addNoise(shrink(big, test_img_scl), rng, 10);

  for (int i=0; i<4; ++i) {
    p[i] *= (1.0/test_img_scl);
  }

  return id;

}

int main(int argc, char** argv) {

  bool debug = false;

  if (argc > 1 && argv[1] == std::string("-d")) {
    debug = true;
  }

  cv::RNG rng(12345);

  TagFamily tagFamily("Tag36h11");

  int dd = tagFamily.d + 2 * tagFamily.blackBorder;

  TPointArray tpoints;

  for (int i=-1; i<=dd; ++i) {
    at::real fi = at::real(i+0.5) / dd;
    for (int j=-1; j<=dd; ++j) {
      at::real fj = at::real(j+0.5) / dd;
      at::real t = -1;
      if (i == -1 || j == -1 || i == dd || j == dd) {
        t = 255; 
      } else if (i == 0 || j == 0 || i+1 == dd || j+1 == dd) {
        t = 0;
      }
      if (t >= 0) {
        tpoints.push_back(TPoint(fi, fj, t));
      }
    }
  }

  int ntrials = 100;
  double total_time = 0;
  double total_err1 = 0;
  double total_err2 = 0;

  for (int trial=0; trial<ntrials; ++trial) {

    TagDetection d;

    at::Point p[4];
    cv::Mat small;

    fakeDetectionImage(tagFamily, rng, p, small);

    cv::Mat_<unsigned char> gsmall;
    cv::cvtColor(small, gsmall, CV_RGB2GRAY);

    at::Mat gx = at::Mat::zeros(gsmall.size());
    at::Mat gy = at::Mat::zeros(gsmall.size());

    /*
    for (int y=1; y<gsmall.rows-1; ++y) {
      for (int x=1; x<gsmall.cols-1; ++x) {
        gx(y,x) = gsmall(y,x+1) - gsmall(y,x-1);
        gy(y,x) = gsmall(y+1,x) - gsmall(y-1,x);
      }
    }
    */

    cv::Sobel( gsmall, gx, at::REAL_IMAGE_TYPE, 1, 0 );
    cv::Sobel( gsmall, gy, at::REAL_IMAGE_TYPE, 0, 1 );

    gx *= 0.25;
    gy *= 0.25;

    at::Point badp[4];

    at::real err1 = 0;
    
    for (int i=0; i<4; ++i) {

      at::Point ei = at::Point( rng.uniform(-1.0,1.0), 
                                rng.uniform(-1.0,1.0) ) * (test_img_tagsz / 8.0);

      badp[i] = p[i] + ei;

      err1 += sqrt(ei.dot(ei));

    }

    total_err1 += err1;
    
    clock_t begin = clock();
    int iter = refineQuad(gsmall, gx, gy, badp, tpoints, debug, 10, 5e-3);
    clock_t end = clock();
    total_time += double(end-begin)/CLOCKS_PER_SEC;

    at::real err2 = 0;
    
    for (int i=0; i<4; ++i) {
      at::Point ei = p[i] - badp[i];
      err2 += sqrt(ei.dot(ei));
    }

    total_err2 += err2;
    
    std::cout << "err before: " << std::setw(10) << err1 
              << ", after: " << std::setw(10) << err2 
              << ", improvement: " << std::setw(10) << err1/err2
              << " after " << iter << " iterations.\n";

  }

  std::cout << "refined " << ntrials << " quads in "
            << total_time << " sec (" << (total_time/ntrials) << " per trial)\n";

  std::cout << "average improvement was " << total_err1 / total_err2 << "\n";

  return 0;

}

    /*

      ucMat img = tagFamily.makeImage(id);
      int b = tagFamily.whiteBorder + tagFamily.blackBorder;

      int errors = 0;
      TagFamily::code_t tagCode = 0;

      for (int iy = tagFamily.d-1; iy >= 0; iy--) {
        for (int ix = 0; ix < tagFamily.d; ix++) {
          
          at::real y = (tagFamily.blackBorder + iy + .5) / dd;
          at::real x = (tagFamily.blackBorder + ix + .5) / dd;
        
          at::Point uv(x, y);
          at::Point pi = interpolate(badp, uv);
          cv::Point ii = pi + at::Point(0.5, 0.5);
        
          int tagbit = img(tagFamily.d-iy-1+b, ix+b) ? 1 : 0;
          int imgbit = gbig(ii) > 127;

          tagCode = tagCode << TagFamily::code_t(1);
          if (imgbit) { 
            tagCode |= TagFamily::code_t(1);
          }
        
          errors += (tagbit != imgbit);
        
          cv::Scalar tc = tagbit ? CV_RGB(255,0,0) : CV_RGB(0,0,255);
          cv::Scalar dc = imgbit ? CV_RGB(255,0,0) : CV_RGB(0,0,255);
          drawPoint(big, pi, tc, 5, CV_FILLED);
          drawPoint(big, pi, dc, 5, 2);

        }
      }

    */
    
