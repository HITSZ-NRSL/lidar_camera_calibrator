/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 ********************************************************************/

#include "TagDetector.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "CameraUtil.h"
#include <map>
#include <stdio.h>
#include <getopt.h>

#ifdef __APPLE__
#include <Glut/Glut.h>
#else
#include <GL/glut.h>
#endif

#define DEFAULT_TAG_FAMILY "Tag36h11"

typedef struct GLTestOptions {
  GLTestOptions() :
      params(),
      family_str(DEFAULT_TAG_FAMILY),
      error_fraction(1),
      device_num(0),
      focal_length(500),
      tag_size(0.1905),
      frame_width(0),
      frame_height(0),
      mirror_display(true)
  {
  }
  TagDetectorParams params;
  std::string family_str;
  double error_fraction;
  int device_num;
  double focal_length;
  double tag_size;
  int frame_width;
  int frame_height;
  bool mirror_display;
} GLTestOptions;


void print_usage(const char* tool_name, FILE* output=stderr) {

  TagDetectorParams p;
  GLTestOptions o;

  fprintf(output, "\
Usage: %s [OPTIONS]\n\
Run a tool to test tag detection. Options:\n\
 -h              Show this help message.\n\
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
 -e FRACTION     Set error detection fraction (default %f)\n\
 -d DEVICE       Set camera device number (default %d)\n\
 -F FLENGTH      Set the camera's focal length in pixels (default %f)\n\
 -z SIZE         Set the tag size in meters (default %f)\n\
 -W WIDTH        Set the camera image width in pixels\n\
 -H HEIGHT       Set the camera image height in pixels\n\
 -M              Toggle display mirroring\n",
          tool_name,
          p.sigma,
          p.segSigma,
          p.thetaThresh,
          p.magThresh,
          p.adaptiveThresholdValue,
          p.adaptiveThresholdRadius,
          DEFAULT_TAG_FAMILY,
          o.error_fraction,
          o.device_num,
          o.focal_length,
          o.tag_size);


  fprintf(output, "Known tag families:");
  TagFamily::StringArray known = TagFamily::families();
  for (size_t i = 0; i < known.size(); ++i) {
    fprintf(output, " %s", known[i].c_str());
  }
  fprintf(output, "\n");
}

GLTestOptions parse_options(int argc, char** argv) {
  GLTestOptions opts;
  const char* options_str = "hDS:s:a:m:V:N:brnf:e:d:F:z:W:H:M";
  int c;
  while ((c = getopt(argc, argv, options_str)) != -1) {
    switch (c) {
      // Reminder: add new options to 'options_str' above and print_usage()!
      case 'h': print_usage(argv[0], stdout); exit(0); break;
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
      case 'd': opts.device_num = atoi(optarg); break;
      case 'F': opts.focal_length = atof(optarg); break;
      case 'z': opts.tag_size = atof(optarg); break;
      case 'W': opts.frame_width = atoi(optarg); break;
      case 'H': opts.frame_height = atoi(optarg); break;
      case 'M': opts.mirror_display = !opts.mirror_display; break;
      default:
        fprintf(stderr, "\n");
        print_usage(argv[0], stderr);
        exit(1);
    }
  }
  opts.params.adaptiveThresholdRadius += (opts.params.adaptiveThresholdRadius+1) % 2;
  return opts;
}


void check_opengl_errors(const char* context) {
  GLenum error = glGetError();
  if (!context) { context = "error"; }
  if (error) {
    std::cerr << context << ": " << gluErrorString(error) << "\n";
  }
}



cv::VideoCapture* capture = 0;

GLuint camera_texture;
std::map<int, GLuint> tag_textures;

cv::Mat frame;
cv::Mat_<cv::Vec3b> uframe;
std::vector<unsigned char> rgbbuf;

GLTestOptions opts;

TagDetector* detector = 0;
TagFamily family;
TagDetectionArray detections;

int width = 640;
int height = 480;

//double f = 500;
//double s = 0.01;

// wb = 1, bb = 1, offs = 1.5/d
// wb = 2, bb = 1, offs = 3/d

//int n = 0;//family.getTagRenderDimension();
//double 

void init() {

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glLineWidth(2.0);
  glEnable(GL_LINE_SMOOTH);

  glGenTextures(1, &camera_texture);
  glBindTexture(GL_TEXTURE_2D, camera_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  
  check_opengl_errors("init");

}

void drawCube() {


  double ss = 0.5*opts.tag_size;
  double sz = opts.tag_size;

  enum { npoints = 8, nedges = 12 };

  cv::Point3d src[npoints] = {
    cv::Point3d(-ss, -ss, 0),
    cv::Point3d( ss, -ss, 0),
    cv::Point3d( ss,  ss, 0),
    cv::Point3d(-ss,  ss, 0),
    cv::Point3d(-ss, -ss, sz),
    cv::Point3d( ss, -ss, sz),
    cv::Point3d( ss,  ss, sz),
    cv::Point3d(-ss,  ss, sz),
  };

  int edges[nedges][2] = {

    { 0, 1 },
    { 1, 2 },
    { 2, 3 },
    { 3, 0 },

    { 4, 5 },
    { 5, 6 },
    { 6, 7 },
    { 7, 4 },

    { 0, 4 },
    { 1, 5 },
    { 2, 6 },
    { 3, 7 }

  };

  glBegin(GL_LINES);

  for (int i=0; i<nedges; ++i) {
    glVertex3dv(&src[edges[i][0]].x);
    glVertex3dv(&src[edges[i][1]].x);
  }

  glEnd();

}

void setupFrustum(double fx, double fy, double cx, double cy,
                  double width, double height,
                  double n, double f) {
  
  double fwd[4][4];

  fwd[0][0] = 2*fx / width;
  fwd[2][0] = -(2*cx - width) / width;
  
  fwd[1][1] = 2*fy / height;
  fwd[2][1] =  (2*cy - height) / height;
  
  fwd[2][2] = -(f+n)/(f-n);
  fwd[3][2] = -(2*f*n)/(f-n);
  
  fwd[2][3] = -1;
  fwd[3][3] = 0;

  glMultMatrixd(&(fwd[0][0]));
  glScaled(opts.mirror_display ? -1 : 1, -1, -1);

}

void bindTexture(int id) {

  std::map<int, GLuint>::const_iterator i = tag_textures.find(id);
  
  if (i != tag_textures.end()) {

    glBindTexture(GL_TEXTURE_2D, i->second);

  } else {

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    tag_textures[id] = tex;

    cv::Mat_<unsigned char> img = family.makeImage(id);
    rgbbuf.resize(img.rows*img.cols*4);

    int offs=0;
    for (int y=0; y<img.rows; ++y) {
      for (int x=0; x<img.cols; ++x) {
        rgbbuf[offs++] = img(y,x);
        rgbbuf[offs++] = img(y,x);
        rgbbuf[offs++] = img(y,x);
        rgbbuf[offs++] = 255;
      }
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 img.cols, img.rows, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, &(rgbbuf[0]));

    check_opengl_errors("texture");

  }

}

void display() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();

  if (opts.mirror_display) {
    gluOrtho2D(width, 0, height, 0);
  } else {
    gluOrtho2D(0, width, height, 0);
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, camera_texture);
  glColor3ub(255,255,255);

  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);  glVertex2f(0, 0);
  glTexCoord2f(1, 0);  glVertex2f(width, 0);
  glTexCoord2f(1, 1);  glVertex2f(width,  height);
  glTexCoord2f(0, 1);  glVertex2f(0,  height);
  glEnd();

  for (size_t i=0; i<detections.size(); ++i) {

    const TagDetection& d = detections[i];

    bindTexture(d.id);

    double k = 1 + (1.5*family.whiteBorder)/family.d;

    cv::Point2d p0 = d.interpolate(-k, -k);
    cv::Point2d p1 = d.interpolate( k, -k);
    cv::Point2d p2 = d.interpolate( k,  k);
    cv::Point2d p3 = d.interpolate(-k,  k);

    glBegin(GL_QUADS);

    glTexCoord2f(0, 1); glVertex2dv(&p0.x);
    glTexCoord2f(1, 1); glVertex2dv(&p1.x);
    glTexCoord2f(1, 0); glVertex2dv(&p2.x);
    glTexCoord2f(0, 0); glVertex2dv(&p3.x);

    glEnd();


  }

  glDisable(GL_TEXTURE_2D);

  glPopMatrix();
  glPushMatrix();

  setupFrustum(opts.focal_length, opts.focal_length,
               width*0.5, height*0.5,
               width, height, 0.01, 10);

  glColor3ub(0,255,0);
  glMatrixMode(GL_MODELVIEW);
  for (size_t i=0; i<detections.size(); ++i) {
    
    cv::Mat_<double> r, R, t, M = cv::Mat_<double>::eye(4,4);
    CameraUtil::homographyToPoseCV(opts.focal_length, opts.focal_length, 
                                   opts.tag_size,
                                   detections[i].homography,
                                   r, t);
    cv::Rodrigues(r, R);

    for (int i=0; i<3; ++i) {
      for (int j=0; j<3; ++j) {
        M(i,j) = R(j,i);
      }
      M(3,i) = t(i);
    }

    glPushMatrix();
    glMultMatrixd(&(M(0,0)));
    drawCube();
    glPopMatrix();

  }
  
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glutSwapBuffers();

  check_opengl_errors("display");


}

void reshape(int w, int h) {

  width = w;
  height = h;

  glClearColor(1,1,1,1);
  glViewport(0,0,w,h);

  check_opengl_errors("reshape");

}

void keyboard(unsigned char value, int x, int y) {

  switch (value) {
  case 27: // esc
    detector->reportTimers();
    exit(1);
    break;
  case 'f':
  case 'F':
    std::cout << "flip!\n";
    opts.mirror_display = !opts.mirror_display;
    break;
  }


}

void idle() {

  *capture >> frame;

  if (frame.type() != CV_8UC3) {
    std::cerr << "bad frame!\n";
    exit(1);
  }

  cv::Point2d opticalCenter(0.5*frame.cols, 0.5*frame.rows);
  detector->process(frame, opticalCenter, detections);
  //std::cout << "got " << detections.size() << " detections.\n";

  uframe = frame;


  rgbbuf.resize(frame.cols*frame.rows*3);

  int offs = 0;

  for (int y=0; y<frame.rows; ++y) {
    for (int x=0; x<frame.cols; ++x) {
      rgbbuf[offs++] = uframe(y,x)[0];
      rgbbuf[offs++] = uframe(y,x)[1];
      rgbbuf[offs++] = uframe(y,x)[2];
    }
  }

  glBindTexture(GL_TEXTURE_2D, camera_texture);

  glTexImage2D(GL_TEXTURE_2D, 0, 3,
               frame.cols, frame.rows, 0, GL_BGR,
               GL_UNSIGNED_BYTE, &(rgbbuf[0]));

  glutPostRedisplay();

  check_opengl_errors("idle");

}

int main(int argc, char** argv) {


  glutInit(&argc, argv);

  opts = parse_options(argc, argv);

  family.init(opts.family_str);
  family.whiteBorder = 1;

  if (opts.error_fraction >= 0 && opts.error_fraction <= 1) {
    family.setErrorRecoveryFraction(opts.error_fraction);
  }

  capture = new cv::VideoCapture(opts.device_num);

  if (opts.frame_width && opts.frame_height) {

    // Use uvcdynctrl to figure this out dynamically at some point?
    capture->set(CV_CAP_PROP_FRAME_WIDTH, opts.frame_width);
    capture->set(CV_CAP_PROP_FRAME_HEIGHT, opts.frame_height);
    

  }

  cv::Mat frame;
  *capture >> frame;

  width = frame.cols;
  height = frame.rows;

  std::cout << "Set camera to resolution: "
            << width << "x"
            << height << "\n";

  detector = new TagDetector(family, opts.params);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | 
                      GLUT_RGB | GLUT_MULTISAMPLE);

  glutInitWindowPosition(100, 100);
  glutInitWindowSize(width, height);

  glutCreateWindow("AprilTags GL Demo");

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  
  init();
  glutMainLoop();

  return 0;

}
