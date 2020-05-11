/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer_ros/ros_map.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_filestem, "map", "Stem of the output files.");
DEFINE_double(resolution, 0.05, "Resolution of a grid cell in the drawn map.");


#define TEST_CARIO
#ifdef TEST_CARIO
DEFINE_double(rot, 30.0, "rot angle (in degree)");
DEFINE_double(trans_x, 0, "trans_x");
DEFINE_double(trans_y, 0, "trans_x");
DEFINE_double(outer_trans_x, 0, "outer_trans_x");
DEFINE_double(outer_trans_y, 0, "outer_trans_y");
#endif


namespace cartographer_ros {
namespace {

#ifdef TEST_CARIO
void testCario() {
  using namespace cartographer::io;
  auto surface = MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(kCairoFormat, 200, 300));
  auto cr = MakeUniqueCairoPtr(cairo_create(surface.get()));
  float f_back_alpha = 1.0, f_back_r = 0.75;
  cairo_set_source_rgba(cr.get(), f_back_r, 0.0, 0.0, f_back_alpha);
  cairo_paint(cr.get());
  cairo_translate(cr.get(), FLAGS_outer_trans_x, FLAGS_outer_trans_y);
  //cairo_scale(cr.get(), 0.5, 0.5);

  cairo_save(cr.get());

  Eigen::Isometry2d homo;
  homo = Eigen::Rotation2Dd(FLAGS_rot*M_PI/180).toRotationMatrix();
  homo.translation() = Eigen::Vector2d(FLAGS_trans_x,FLAGS_trans_y);  
  std::ostringstream oss;
  oss << homo.matrix();
  LOG(INFO) << "homo: \n" << oss.str();
  
  cairo_matrix_t matrix;
  cairo_matrix_init(&matrix, homo(0,0), homo(1, 0), homo(0, 1), homo(1, 1),
                    homo(0, 2), homo(1, 2));
  cairo_transform(cr.get(), &matrix);
  //cairo_scale(cr.get(), 2, 2);

  double x=100.0;
  double y=0;
  Eigen::Vector2d eigen_transformed = homo * Eigen::Vector2d(x, y);
  LOG(INFO) << "point in user  : " << x << ", " << y;
  LOG(INFO) << "point transformed by eigen  : " << eigen_transformed[0] << ", " << eigen_transformed[1];
  cairo_user_to_device(cr.get(), &x, &y);
  LOG(INFO) << "point in device: " << x << ", " << y;

  int width = 100;
  int height = 150;
  std::vector<char> intensity;
  std::vector<char> alpha;
  std::vector<uint32_t> cairo_data;
  std::ostringstream oss2;
  std::ostringstream oss3;
  for (int i=0; i<height; i++) {
    for (int j=0; j<width; j++) {
      int a = (i*4/height + j*4/width) * 32;
      if (a==256) {
        a=255;
      }
      int delta = 128 - a;
      uint8_t tmp_alpha = delta > 0 ? 0 : -delta;
      uint8_t tmp_value = delta > 0 ? delta : 0;
      intensity.push_back(tmp_value);
      tmp_alpha = (tmp_value || tmp_alpha) ? tmp_alpha : 1;
      alpha.push_back(tmp_alpha);
      float f_final_alpha = (tmp_alpha/255.0) + f_back_alpha * (1-(tmp_alpha/255.0));
      //float f_final_value = ((tmp_value/255.0)*(tmp_alpha/255.0) + f_back_r * f_back_alpha * (1-(tmp_alpha/255.0))) / f_final_alpha;
      float f_final_value = (tmp_value/255.0) + f_back_r * f_back_alpha * (1-(tmp_alpha/255.0));
      if (f_final_value > 1.0) {
        f_final_value = 1.0;
      }
      uint8_t final_alpha = int(f_final_alpha * 255);
      uint8_t final_value = int(f_final_value * 255);

      if (i%10==0 && j%10==0) {        
        oss2 << int(tmp_alpha) << "|" << int(tmp_value) << "\t";
      }
      if (i%20==0 && j%20==0) {
        oss3 << int(final_alpha) << "|" << int(final_value) << "\t";
      }
    }
    if (i%10==0) {
      oss2 << "\n";
    }
    if (i%20==0) {
      oss3 << "\n";
    }
  }
  LOG(INFO) << "10-down-sampled inner-surface data\n" << oss2.str();
  LOG(INFO) << "20-down-sampled inner-surface combined-data\n" << oss3.str();

  auto inner_surface = DrawTexture(intensity, alpha, width, height, &cairo_data);
  cairo_surface_flush(inner_surface.get());

  cairo_set_source_surface(cr.get(), inner_surface.get(), 0., 0.);
  cairo_paint(cr.get());

  {
    uint8_t* data = cairo_image_surface_get_data(surface.get());
    int	width = cairo_image_surface_get_width (surface.get());
    int	height = cairo_image_surface_get_height (surface.get());
    int	stride = cairo_image_surface_get_stride (surface.get());
    std::ostringstream oss4;
    for (int i=0; i<height; i+=20) {
      for (int j=0; j<width; j+=20) {
        uint8_t r = data[i*stride + j*4 + 2];
        uint8_t a = data[i*stride + j*4 + 3];
        oss4 << int(a) << "|" << int(r) << "\t";
      }
      oss4 << "\n";
    }
    LOG(INFO) << "20-down-sampled outer surface data\n" << oss4.str();
  }

  cairo_restore(cr.get());


  cairo_surface_flush(surface.get());
  //cairo_surface_write_to_png (surface.get(), "cario_test.png");


  ::cartographer::io::StreamFileWriter pgm_writer("cario_test.pgm");
  ::cartographer::io::Image image(std::move(surface));
  double resolution = 0.05;
  WritePgm(image, resolution, &pgm_writer);
}
#endif


void Run(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;
  ::cartographer::io::DeserializeAndFillSubmapSlices(
      &deserializer, &submap_slices, &conversion_tables);
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

#ifdef TEST_CARIO
  ::cartographer_ros::testCario();
  return 0;
#endif

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_map_filestem.empty()) << "-map_filestem is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_filestem,
                          FLAGS_resolution);
}
