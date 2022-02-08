/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 ********************************************************************/

#include "TagFamily.h"
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <ctype.h>

struct Unit {
  std::string name;
  double points;
};

enum {
  num_units = 5
};

static const Unit units[num_units] = {
  { "pt", 1 },
  { "in", 72 },
  { "ft", 72*12 },
  { "cm", 28.346456693 },
  { "mm", 2.8346456693 },
};

static int last_unit = 0;

struct PaperSize {
  std::string name;
  double width_points;
  double height_points;
};

static const PaperSize papers[] = {
  { "letter", 612, 792 },
  { "a6", 297.5, 419.6 },
  { "a5", 419.3, 595.4 },
  { "a4", 595, 842 },
  { "a3", 841.5, 1190.7 },
  { "a2", 1190, 1684 },
  { "a1", 1683, 2384.2 },
  { "a0", 2382.8, 3370.8 },
  { "", 0 }
};

int lookup_unit(const std::string& label) {

  for (int i=0; i<num_units; ++i) {
    if (units[i].name == label) {
      last_unit = i;
      return i;
    }
  }

  std::cerr << "invalid unit name " << label << "\n";
  exit(1);
  
}


double parse_unit(std::istream& istr) {

  double rval;

  if (!(istr>>rval)) {
    std::cerr << "error: expected number\n";
    exit(1);
  }

  if (istr.eof()) { 
    return rval; 
  }

  int i = istr.peek();

  while (isspace(i)) { 
    istr.get();
    i = istr.peek();
  }

  if (isalpha(i)) {
    std::string label;
    istr >> label;
    int i = lookup_unit(label);
    rval *= units[i].points;  
  }

  return rval;
  
};

double parse_unit(const std::string& str) {
  std::istringstream istr(str);
  return parse_unit(istr);
}

template <class Tval>
Tval parse_value(const std::string& str) {
  std::istringstream istr(str);
  Tval rval;
  if (!(istr >> rval) || istr.peek() != EOF) {
    std::cerr << "error parsing value\n";
    exit(1);
  } 
  return rval;
}

void usage(std::ostream& ostr) {

  ostr << "usage: maketags FAMILY [OPTIONS]\n\n"
       << "  Known tag families:"; 

  TagFamily::StringArray known = TagFamily::families();
  for (size_t i=0; i<known.size(); ++i) { ostr << " " << known[i]; }
  ostr << "\n\n";

  ostr << "Options:\n\n"
       << "   --paper TYPE               where TYPE is one of: ";
  for (int i=0; papers[i].width_points; ++i) { ostr << papers[i].name << " "; }
  ostr << "\n";
  ostr << "   --paperdims LENGTH LENGTH  dimensions of paper\n"
       << "   --margin LENGTH            set page margin\n"
       << "   --margins T B L R          set page margins to given lengths\n"
       << "   --tagsize LENGTH           width of entire tag including white border\n"
       << "   --innersize LENGTH         width of black portion of tag only\n"
       << "   --tagfrac N                generate N tags per paper width (set tag size automatically)\n"
       << "   --padding LENGTH           set padding between tags\n"
       << "   --id N                     generate only tag with id N (mutliple allowed)\n"
       << "   --idrange N1 N2            generate only tags with id's between N1 and N2 (inclusive)\n"
       << "   --label                    add labels\n"
       << "   --labelsize LENGTH         label font size\n"
       << "   --labelgray G              label darkness (0-1)\n"
       << "   --labelfmt FMT             label format %d=id, %f=family, %i=innersize %t=tagsize\n"
       << "   --cropmark X Y L G         emit cropmarks at (X,Y) from edge with length L and gray G\n"
       << "   --outfile FILENAME.pdf     output filename (include pdf extension)\n"
       << "   --help                     see this message\n"
       << "\n";
  ostr << "LENGTH may be specified using units: ";
  for (int i=0; i<num_units; ++i) { ostr << units[i].name << " "; }
  ostr << "\n\n";

 
}

int main(int argc, char** argv) {

  enum SizeType {
    SizeFull,
    SizeInner,
    SizeFrac,
  };

  enum MarginType {
    MT=0, MB, ML, MR
  };
  
  double width = 612;
  double height = 792;
  SizeType stype = SizeFull;
  double size = width;
  double tagfrac = 1;
  double padding = 0;
  double margins[4] = { 36, 36, 36, 36 }; // 0.5in margins all around
  double font_size = 0;
  double label_gray = 0;
  double crop_x = 0;
  double crop_y = 0;
  double crop_l = 36;
  double crop_w = 1;
  double crop_gray = 0.8;
  std::string outfile = "pdffile.pdf";
  bool   do_crop = false;

  std::string label_fmt = "%d";

  int label_unit = -1;

  bool draw_labels = false;

  if (argc < 2) {
    usage(std::cerr);
    exit(1);
  }
  
  std::vector<size_t> ids;
  
  if (std::string(argv[1]) == "--help") {
    usage(std::cout);
    exit(0);
  }

  TagFamily family(argv[1]);

  for (int i=2; i<argc; ++i) {
    std::string optarg = argv[i];
    if (optarg == "--paper") {
      std::string paper = argv[++i];
      bool found = false;
      for (int j=0; papers[j].width_points; ++j) {
        if (papers[j].name == paper) {
          found = true;
          width = papers[j].width_points;
          height = papers[j].height_points;
        }
      }
      if (!found) {
        std::cerr << "unrecognized paper size: " << paper << "\n";
        exit(1);
      }
    } else if (optarg == "--papersize") {
      width = parse_unit(argv[++i]);
      height = parse_unit(argv[++i]);
    } else if (optarg == "--margin") {
      double l = parse_unit(argv[++i]);
      for (int j=0; j<4; ++j) { margins[j] = l; }
    } else if (optarg == "--margins") {
      for (int j=0; j<4; ++j) { margins[j] = parse_unit(argv[++i]); }
    } else if (optarg == "--label") {
      draw_labels = true;
    } else if (optarg == "--labelsize") {
      font_size = parse_unit(argv[++i]);
    } else if (optarg == "--labelgray") {
      label_gray = parse_value<double>(argv[++i]);
    } else if (optarg == "--labelfmt") {
      label_fmt = argv[++i];
    } else if (optarg == "--tagsize") {
      size = parse_unit(argv[++i]);
      stype = SizeFull;
      label_unit = last_unit;
    } else if (optarg == "--innersize") {
      size = parse_unit(argv[++i]);
      stype = SizeInner;
      label_unit = last_unit;
    } else if (optarg == "--cropmark") {
      do_crop = true;
      crop_x = parse_unit(argv[++i]);
      crop_y = parse_unit(argv[++i]);
      crop_l = parse_unit(argv[++i]);
      crop_gray = parse_value<double>(argv[++i]);
    } else if (optarg == "--padding") {
      padding = parse_unit(argv[++i]);
    } else if (optarg == "--tagfrac") {
      tagfrac = parse_value<double>(argv[++i]);
      stype = SizeFrac;
      if (tagfrac < 1) {
        std::cerr << "tag fraction must be greater than one \n";
        exit(1);
      }
    } else if (optarg == "--id") {
      ids.push_back(parse_value<size_t>(argv[++i]));
    } else if (optarg == "--idrange") {
      size_t start = parse_value<size_t>(argv[++i]);
      size_t end = parse_value<size_t>(argv[++i]);
      for (size_t i=start; i<=end; ++i) {
        ids.push_back(i);
      }
    } else if (optarg == "--outfile") {
      outfile = argv[++i];
    } else if (optarg == "--help") {
      usage(std::cout);
      exit(0);
    } else {
      std::cerr << "unrecognized option: " << optarg << "\n";
      exit(1);
    }
  }

  if (label_unit < 0) { label_unit = lookup_unit("mm"); }

  int tags_per_row = 0;
  double sq_size = 0;

  if (width <= margins[ML] + margins[MR]) {
    std::cerr << "width must be greater than L+R margins\n";
    exit(1);
  } else if (height <= margins[MT] + margins[MB]) {
    std::cerr << "height must be greater than T+B margins\n";
    exit(1);
  }

  double orig_width = width;
  double orig_height = height;

  width -= margins[ML] + margins[MR];
  height -= margins[MT] + margins[MB];

  int rd = family.getTagRenderDimension();
  int id = rd - 2*family.whiteBorder;

  if (stype == SizeInner) {
    sq_size = size;
    size = sq_size * double(rd)/id;
  } else if (stype == SizeFrac) {
    size = (width - (tagfrac-1)*padding) / tagfrac;
    tags_per_row = int(tagfrac);
  }

  if (!sq_size) { sq_size = size * double(id)/rd; }

  if (!tags_per_row) { 

    if (size > width) {
      std::cerr << "tag size wider than page!\n";
    }
    tags_per_row = 1 + (width - size) / (padding + size);

  }

  double px_size = size / rd;

  double row_size = size;

  if (font_size == 0) {
    font_size = px_size;
  }

  if (draw_labels) {
    row_size += font_size;
  }

  if (row_size > height) {
    std::cerr << "tag size taller than page!\n";
    exit(1);
  }

  int rows_per_page = 1 + int((height - row_size) / (padding + row_size));

  int tags_per_page = tags_per_row * rows_per_page;

  std::vector<size_t> tmp;
  for (size_t i=0; i<ids.size(); ++i) {
    if (ids[i] < family.codes.size()) { tmp.push_back(ids[i]); }
  }
  ids.swap(tmp);

  std::cout << "paper size: " << orig_width << "x" << orig_height << "\n";
  std::cout << "imagable area: " << width << "x" << height << "\n";
  std::cout << "tag size: " << size << "\n";
  std::cout << "padding size: " << padding << "\n";

  if (ids.empty()) {
    std::cout << "all ids\n";
    for (size_t i=0; i<family.codes.size(); ++i) { ids.push_back(i); }
  } else {
    std::cout << "ids: ";
    for (size_t i=0; i<ids.size(); ++i) { std::cout << ids[i] << " "; }
    std::cout << "\n";
  }
  
  int output_pages = int(ceil(double(ids.size()) / tags_per_page));

  std::cout << "tags per row: " << tags_per_row << "\n";
  std::cout << "rows per page: " << rows_per_page << "\n";
  std::cout << "tags per page: " << tags_per_page << "\n";
  std::cout << "output pages: " << output_pages << "\n";

  std::cout << "pixel size: " << px_size << "\n";

  std::cout << "square size: " << sq_size << "\n";

  /*
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " FAMILY OUTPUTDIR\n";
    std::cerr << "Known tag families:";
    TagFamily::StringArray known = TagFamily::families();
    for (size_t i=0; i<known.size(); ++i) { std::cerr << " " << known[i]; }
    std::cerr << "\n";
    return 1;
  }

  TagFamily family(argv[1]);

  family.writeAllImagesPostScript(argv[2] + std::string("/") + argv[1] + ".ps");
  */


  cairo_surface_t *surface;
  cairo_t *cr;

  surface = cairo_pdf_surface_create(outfile.c_str(), orig_width, orig_height);
  cr = cairo_create(surface);


  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                          CAIRO_FONT_WEIGHT_NORMAL);

  cairo_set_font_size (cr, font_size);

  int row = 0;
  int col = 0;
  bool newpage = false;
  int wb = family.whiteBorder;
  int bb = family.blackBorder;
  int tb = wb + bb;


  double mw = margins[ML] + 0.5 * (width - size*tags_per_row - padding*(tags_per_row-1));
  double mh = margins[MT] + 0.5 * (height - row_size*rows_per_page - padding*(rows_per_page-1));
  
  for (size_t i=0; i<ids.size(); ++i) {

    if (newpage) { 
      cairo_show_page(cr); 
      newpage = false; 
    }

    double x = mw + wb * px_size + col * (size + padding);
    double y = mh + wb * px_size + row * (row_size + padding);

    if (do_crop) {

      cairo_set_line_width(cr, crop_w);

      cairo_set_source_rgb(cr, 
                           crop_gray,
                           crop_gray,
                           crop_gray);

      cairo_new_path(cr);

      double x0 = mw + col * (size + padding);
      double y0 = mh + row * (row_size + padding);
      double x1 = x0 + size;
      double y1 = y0 + size;

      cairo_move_to(cr, x0-crop_x, y0-crop_y);
      cairo_line_to(cr, x0-crop_x-crop_l, y0-crop_y);

      cairo_move_to(cr, x0-crop_x, y0-crop_y);
      cairo_line_to(cr, x0-crop_x, y0-crop_y-crop_l);

      cairo_move_to(cr, x1+crop_x, y0-crop_y);
      cairo_line_to(cr, x1+crop_x+crop_l, y0-crop_y);

      cairo_move_to(cr, x1+crop_x, y0-crop_y);
      cairo_line_to(cr, x1+crop_x, y0-crop_y-crop_l);

      cairo_move_to(cr, x0-crop_x, y1+crop_y);
      cairo_line_to(cr, x0-crop_x-crop_l, y1+crop_y);

      cairo_move_to(cr, x0-crop_x, y1+crop_y);
      cairo_line_to(cr, x0-crop_x, y1+crop_y+crop_l);

      cairo_move_to(cr, x1+crop_x, y1+crop_y);
      cairo_line_to(cr, x1+crop_x+crop_l, y1+crop_y);

      cairo_move_to(cr, x1+crop_x, y1+crop_y);
      cairo_line_to(cr, x1+crop_x, y1+crop_y+crop_l);
      
      cairo_stroke(cr);

    }


    // draw thing at x, y
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_new_path(cr);
    cairo_rectangle(cr, x, y, sq_size, sq_size);
    cairo_fill(cr);

    if (draw_labels) {

      cairo_set_source_rgb(cr, 
                           label_gray, 
                           label_gray, 
                           label_gray);

      //char buf[1024];
      //snprintf(buf, 1024, "%u", (unsigned int)ids[i]);
      int id = ids[i];
      std::string lstr = label_fmt;

      // replace %[0-9]+d with buf
      // replace %f with stuff
      size_t pos;
      size_t start = 0;
      while ( (pos = lstr.find('%', start)) != std::string::npos ) {

        size_t pos2 = pos+1;
        while (pos2 < lstr.length() && isdigit(lstr[pos2])) {
          ++pos2;
        }

        if (pos2 >= lstr.length()) { break; }

        size_t l = pos2-pos + 1;

        char buf[1024];
        
        if (tolower(lstr[pos2]) == 'f') {
          lstr.replace(pos, l, argv[1]);
        } else if (tolower(lstr[pos2]) == 'i' ||
                   tolower(lstr[pos2]) == 't' ||
                   tolower(lstr[pos2]) == 'p') {
          double meas;
          switch (tolower(lstr[pos2])) {  
          case 'i':
            meas = sq_size;
            break;
          case 't':
            meas = size;
            break;
          default:
            meas = px_size;
            break;
          }
          meas /= units[label_unit].points;
          snprintf(buf, 1024, "%g %s", meas, units[label_unit].name.c_str());
          lstr.replace(pos, l, buf);
        } else if (tolower(lstr[pos2]) == 'd') {
          snprintf(buf, 1024, lstr.substr(pos,l).c_str(), id);
          lstr.replace(pos, l, buf);
        }

        start = pos2;

      }
      
      

      

      cairo_text_extents_t extents;
      cairo_text_extents (cr, lstr.c_str(), &extents);

      double tx = 0.5 * (sq_size - extents.width);
      cairo_move_to(cr, x + tx, y + sq_size + px_size * wb + font_size);
      cairo_show_text(cr, lstr.c_str());

    }

    /*
    cairo_rectangle(cr, x, y + sq_size + px_size * wb, sq_size, font_size);
    cairo_fill(cr);
    */

    cv::Mat_<unsigned char> m = family.makeImage(ids[i]);

    cairo_set_source_rgb(cr, 1, 1, 1);

    for (size_t i=0; i<family.d; ++i) {
      for (size_t j=0; j<family.d; ++j) {
        bool w = m(i+tb,j+tb);
        if (w) {
          cairo_set_line_width(cr, px_size / 256);
          cairo_new_path(cr);
          cairo_rectangle(cr, x+(j+bb)*px_size, y+(i+bb)*px_size, px_size, px_size);
          //cairo_save(cr);
          cairo_fill_preserve(cr);
          //cairo_restore(cr);
          cairo_stroke(cr);
        }
      }
    }


    ++col;
    if (col >= tags_per_row) {
      col = 0;
      ++row;
      if (row >= rows_per_page) {
        row = 0;
        newpage = true;
      }
    }

  }

  cairo_show_page(cr);
  cairo_surface_destroy(surface);
  cairo_destroy(cr);

  return 0;


}
