/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 ********************************************************************/

#include <cstdio>
#include <iostream>
#include <getopt.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TagDetector.h"
#include "DebugImage.h"

#define DEFAULT_TAG_FAMILY "Tag36h11"

typedef struct TagTestOptions {
  TagTestOptions() :
      show_debug_info(false),
      show_timing(false),
      show_results(false),
      be_verbose(false),
      no_images(false),
      generate_output_files(false),
      params(),
      family_str(DEFAULT_TAG_FAMILY),
      error_fraction(1){
  }
  bool show_debug_info;
  bool show_timing;
  bool show_results;
  bool be_verbose;
  bool no_images;
  bool generate_output_files;
  TagDetectorParams params;
  std::string family_str;
  double error_fraction;
} TagTestOptions;


void print_usage(const char* tool_name, FILE* output=stderr) {

  TagDetectorParams p;

  fprintf(output, "\
Usage: %s [OPTIONS] IMAGE1 [IMAGE2 ...]\n\
Run a tool to test tag detection. Options:\n\
 -h              Show this help message.\n\
 -d              Show debug images and data during tag detection.\n\
 -t              Show timing information for tag detection.\n\
 -R              Show textual results of tag detection.\n\
 -v              Be verbose (includes -d -t -R).\n\
 -x              Do not generate any non-debug visuals.\n\
 -o              Generate debug visuals as output files vs. using X11.\n\
 -D              Use decimation for segmentation stage.\n\
 -S SIGMA        Set the original image sigma value (default %.2f).\n\
 -s SEGSIGMA     Set the segmentation sigma value (default %.2f).\n\
 -a THETATHRESH  Set the theta threshold for clustering (default %.1f).\n\
 -m MAGTHRESH    Set the magnitude threshold for clustering (default %.1f).\n\
 -V VALUE        Set adaptive threshold value for new quad algo (default %f).\n\
 -N RADIUS       Set adaptive threshold radius for new quad algo (default %d).\n\
 -b              Refine bad quads using template tracker.\n\
 -r              Refine all quads using template tracker.\n\
 -n              Use the new quad detection algorithm.\n\
 -f FAMILY       Look for the given tag family (default \"%s\")\n\
 -e FRACTION     Set error detection fraction (default 1)\n",
          tool_name,
          p.sigma,
          p.segSigma,
          p.thetaThresh,
          p.magThresh,
          p.adaptiveThresholdValue,
          p.adaptiveThresholdRadius,
          DEFAULT_TAG_FAMILY);

  fprintf(output, "Known tag families:");
  TagFamily::StringArray known = TagFamily::families();
  for (size_t i = 0; i < known.size(); ++i) {
    fprintf(output, " %s", known[i].c_str());
  }
  fprintf(output, "\n");
}

TagTestOptions parse_options(int argc, char** argv) {
  TagTestOptions opts;
  const char* options_str = "hdtRvxoDS:s:a:m:V:N:brnf:e:";
  int c;
  while ((c = getopt(argc, argv, options_str)) != -1) {
    switch (c) {
      // Reminder: add new options to 'options_str' above and print_usage()!
      case 'h': print_usage(argv[0], stdout); exit(0); break;
      case 'd': opts.show_debug_info = true; break;
      case 't': opts.show_timing = true; break;
      case 'R': opts.show_results = true; break;
      case 'v': opts.be_verbose = true; break;
      case 'x': opts.no_images = true; break;
      case 'o': opts.generate_output_files = true; break;
      case 'D': opts.params.segDecimate = true; break;
      case 'S': opts.params.sigma = atof(optarg); break;
      case 's': opts.params.segSigma = atof(optarg); break;
      case 'a': opts.params.thetaThresh = atof(optarg); break;
      case 'm': opts.params.magThresh = atof(optarg); break;
      case 'V': opts.params.adaptiveThresholdValue = atof(optarg); break;
      case 'N': opts.params.adaptiveThresholdRadius = atoi(optarg); break;
      case 'b': opts.params.refineBad = true; break;
      case 'r': opts.params.refineQuads = true; break;
      case 'n': opts.params.newQuadAlgorithm = true; break;
      case 'f': opts.family_str = optarg; break;
      case 'e': opts.error_fraction = atof(optarg); break;
      default:
        fprintf(stderr, "\n");
        print_usage(argv[0], stderr);
        exit(1);
    }
  }
  opts.params.adaptiveThresholdRadius += (opts.params.adaptiveThresholdRadius+1) % 2;
  if (opts.be_verbose) {
    opts.show_debug_info = opts.show_timing = opts.show_results = true;
  }
  return opts;
}

int main(int argc, char** argv) {

  const std::string win = "Tag test";

  TagTestOptions opts = parse_options(argc, argv);

  TagFamily family(opts.family_str);

  if (opts.error_fraction >= 0 && opts.error_fraction < 1) {
    family.setErrorRecoveryFraction(opts.error_fraction);
  }

  TagDetector detector(family, opts.params);
  detector.debug = opts.show_debug_info;
  detector.debugWindowName = opts.generate_output_files ? "" : win;
  if (opts.params.segDecimate && opts.be_verbose) {
    std::cout << "Will decimate for segmentation!\n";
  }

  TagDetectionArray detections;
  for (int i=optind; i<argc; ++i) {
    cv::Mat src = cv::imread(argv[i]);
    if (src.empty()) { continue; }

    while (std::max(src.rows, src.cols) > 800) {
      cv::Mat tmp;
      cv::resize(src, tmp, cv::Size(0,0), 0.5, 0.5);
      src = tmp;
    }
    cv::Point2d opticalCenter(0.5*src.rows, 0.5*src.cols);

    if (!detector.debug && !opts.no_images) {
      labelAndWaitForKey(win, "Original", src, ScaleNone, true);
    }

    clock_t start = clock();
    detector.process(src, opticalCenter, detections);
    clock_t end = clock();

    if (opts.show_results) {
      if (opts.show_debug_info) std::cout << "\n";
      std::cout << "Got " << detections.size() << " detections in "
                << double(end-start)/CLOCKS_PER_SEC << " seconds.\n";
      for (size_t i=0; i<detections.size(); ++i) {
        const TagDetection& d = detections[i];
        std::cout << " - Detection: id = " << d.id << ", "
                  << "code = " << d.code << ", "
                  << "rotation = " << d.rotation << "\n";
      }
    }
    if (!opts.no_images) {
      cv::Mat img = family.superimposeDetections(src, detections);
      labelAndWaitForKey(win, "Detected", img, ScaleNone, true);
    }
  }

  if (opts.show_timing) detector.reportTimers();
  return 0;
}
