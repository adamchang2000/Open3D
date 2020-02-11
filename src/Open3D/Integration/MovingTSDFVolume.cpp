// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------



//Created by Adam Chang
//Implementation of Joe's moving 3x3 mesh blocks for real-time reconstruction

#include "Open3D/Integration/MovingTSDFVolume.h"
#include <iostream>
#include <chrono>
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"


//#include "Open3D/Geometry/PointCloud.h"
#include "Open3D/Integration/MarchingCubesConst.h"
#include "Open3D/Integration/UniformTSDFVolume.h"
#include "Open3D/Utility/Console.h"

namespace open3d {
	namespace integration {

		MovingTSDFVolume::MovingTSDFVolume(double voxel_length,
                       double sdf_trunc,
                       TSDFVolumeColorType color_type,
                       double block_length)
		:	block_length_(block_length),
			voxel_length_(voxel_length),
			sdf_trunc_(sdf_trunc),
			color_type_(color_type){
				Eigen::Vector3i * temp = &current_block;
				temp = NULL;
				active_volume = new ScalableTSDFVolume(voxel_length_,
                       sdf_trunc_,
                       color_type_);
			}

		MovingTSDFVolume::~MovingTSDFVolume() {}

		void MovingTSDFVolume::Reset() {
			active_volume->Reset();
		}

		void MovingTSDFVolume::Integrate(
			const geometry::RGBDImage &image,
			const camera::PinholeCameraIntrinsic &intrinsic,
			const Eigen::Matrix4d &extrinsic) { //T_WS.inverse()
			
			if ((std::chrono::system_clock::now() - latest_loop_closure).count() < time_threshold) {
				printf("too recent of a loop closure, skipping integration.\n");
				return;
			}

			if (active_volume_keyframe_num == -1) {
				printf("waiting for first keyframe\n");
				return;
			}

			UpdateActiveVolume(extrinsic);

			Eigen::Matrix4d transform_kf_coords = Eigen::Matrix4d::Identity();
			transform_kf_coords = extrinsic * active_volume_transform; //F_WS.inverse()*K_WS --- F_WS*K_WS.inverse() 

			//std::cout << extrinsic << std::endl;
			//std::cout << transform_kf_coords << std::endl;	

			
			active_volume->Integrate(image, intrinsic, transform_kf_coords);

		}

		void MovingTSDFVolume::UpdateActiveVolume(Eigen::Matrix4d extrinsic) {

			//std::lock_guard<std::mutex> guard2(meshLock);

			//locate which block we are in, origin = center of block (l/2, l/2, l/2)
			auto block_loc = LocateBlock(Eigen::Vector3d(extrinsic(0, 3) - block_length_ / 2, extrinsic(1, 3) - block_length_ / 2, extrinsic(2, 3) - block_length_ / 2));

			if (&current_block == NULL) {
				current_block = block_loc;
			}

			//return if we are still in the same current_block
			if (block_loc.isApprox(current_block)) {
				return;
			}
			std::lock_guard<std::mutex> guard(keyFrameLock);

			printf("EXITED BLOCK, NEW BLOCK\n");

			auto completed_mesh = std::make_shared<MeshUnit>();

			auto mesh = active_volume->ExtractTriangleMesh();
			completed_mesh->mesh = mesh;
			completed_mesh->transform = active_volume_transform;
			completed_mesh->keyframe_num = active_volume_keyframe_num;
			completed_mesh->block_loc = current_block;
			completed_meshes.push_back(completed_mesh);



			active_volume = new ScalableTSDFVolume(voxel_length_, sdf_trunc_, color_type_);
			active_volume_transform = latest_keyframe;
			active_volume_keyframe_num = latest_keyframe_num;

			current_block = block_loc;
		}

		void MovingTSDFVolume::CombineMeshes(std::shared_ptr<geometry::TriangleMesh>& output_mesh, std::shared_ptr<geometry::TriangleMesh> mesh_to_combine) {

			//printf("combine meshes called %d\n", output_mesh->vertices_.size());
			//printf("mesh to combine vertices %d\n", mesh_to_combine->vertices_.size());

			/*std::vector<Eigen::Vector3d> out_vertices = output_mesh->vertices_;
			std::vector<Eigen::Vector3d> out_colors = output_mesh->vertex_colors_;
			std::vector<Eigen::Vector3i> out_triangles = output_mesh->triangles_;*/

			std::vector<Eigen::Vector3d> vertices = mesh_to_combine->vertices_;
			std::vector<Eigen::Vector3d> colors = mesh_to_combine->vertex_colors_;
			std::vector<Eigen::Vector3i> triangles = mesh_to_combine->triangles_;

			size_t tri_count = output_mesh->vertices_.size();

			output_mesh->vertices_.insert(output_mesh->vertices_.end(), vertices.begin(), vertices.end());
			output_mesh->vertex_colors_.insert(output_mesh->vertex_colors_.end(), colors.begin(), colors.end());
			for (auto triangle : triangles) {
				Eigen::Vector3i triangle_;
				triangle_(0) = triangle(0) + tri_count;
				triangle_(1) = triangle(1) + tri_count;
				triangle_(2) = triangle(2) + tri_count;
				output_mesh->triangles_.push_back(triangle_);
			}

			//printf("combine meshes exited %d\n", output_mesh->vertices_.size());
		}


		void MovingTSDFVolume::SetLatestKeyFrame(Eigen::Matrix4d transform, int keyframe_num) {
			latest_keyframe = transform;
			latest_keyframe_num = keyframe_num;

			if (active_volume_keyframe_num == -1) {
				printf("init first keyframe\n");
				active_volume_keyframe_num = keyframe_num;
				active_volume_transform = transform;
			}
		}

		void MovingTSDFVolume::UpdateKeyFrames(std::map<int, Eigen::Matrix4d> keyframes) {
			std::lock_guard<std::mutex> guard(keyFrameLock);
			printf("loop closure detected -> key frames updating\n");

			latest_loop_closure = std::chrono::system_clock::now();

			/*std::shared_ptr<open3d::geometry::TriangleMesh> write_mesh = ExtractTotalTriangleMesh();
			open3d::io::WriteTriangleMeshToPLY("mesh_" + std::to_string(counter) + ".ply", *write_mesh, false, false, true, true, false, false);
			counter++;*/

			printf("past meshes count: %d\n", completed_meshes.size());

			for (auto completed_mesh : completed_meshes) {
				Eigen::Matrix4d updated_transform = keyframes[completed_mesh->keyframe_num];
				completed_mesh->transform = updated_transform;
			}

			if (keyframes.count(active_volume_keyframe_num)) {
				active_volume_transform = keyframes[active_volume_keyframe_num];

			}
		}

		//returns a vector of mesh in its own local coords and associated viewing transform
		std::vector<std::pair<std::shared_ptr<geometry::TriangleMesh>, Eigen::Matrix4d>> MovingTSDFVolume::GetTriangleMeshes() {

			//printf("get triangle meshes called\n");

			std::vector<std::pair<std::shared_ptr<geometry::TriangleMesh>, Eigen::Matrix4d>> ret;
			for (auto completed_mesh : completed_meshes) {

				//printf("mesh with sizes: %d %d \n", completed_mesh->mesh->vertices_.size(), completed_mesh->mesh->triangles_.size());

				ret.push_back(std::pair<std::shared_ptr<geometry::TriangleMesh>, Eigen::Matrix4d>(completed_mesh->mesh, completed_mesh->transform));
			}

			ret.push_back(std::pair<std::shared_ptr<geometry::TriangleMesh>, Eigen::Matrix4d>(ExtractCurrentTriangleMesh(), active_volume_transform));

			return ret;
		}



		//todo implement extracting from all the triangle meshes
		std::shared_ptr<geometry::TriangleMesh>
			MovingTSDFVolume::ExtractTotalTriangleMesh() {

				printf("extracting triangle meshes\n");

				auto mesh_output = std::make_shared<geometry::TriangleMesh>();

				for (auto mesh_unit : completed_meshes) {
					
					//printf("adding a mesh\n");

					auto completed_mesh = mesh_unit->mesh;
					auto transformed_mesh = std::make_shared<geometry::TriangleMesh>();

					auto vertices = completed_mesh->vertices_;
					std::vector<Eigen::Vector3d> transformed_vertices;

					for (Eigen::Vector3d vertex : vertices) {

						Eigen::Vector4d v(vertex(0), vertex(1), vertex(2), 1.0);
						v = mesh_unit->transform * v;
						Eigen::Vector3d transformed_v(v(0), v(1), v(2));

						transformed_vertices.push_back(transformed_v);
					}

					transformed_mesh->vertices_ = transformed_vertices;
					transformed_mesh->vertex_colors_ = completed_mesh->vertex_colors_;
					transformed_mesh->triangles_ = completed_mesh->triangles_;

					CombineMeshes(mesh_output, transformed_mesh);

				}

				
				auto mesh = active_volume->ExtractTriangleMesh();
				for (int i = 0; i < mesh->vertices_.size(); ++i) {
					Eigen::Vector4d v(mesh->vertices_[i](0), mesh->vertices_[i](1), mesh->vertices_[i](2), 1.0);
					v = active_volume_transform * v;
					Eigen::Vector3d transformed_v(v(0), v(1), v(2));
					mesh->vertices_[i] = transformed_v;
				}

				CombineMeshes(mesh_output, mesh);

				return mesh_output;
		}

		std::shared_ptr<geometry::TriangleMesh>
			MovingTSDFVolume::ExtractCurrentTriangleMesh() {
				if (active_volume == NULL)
					utility::LogError("No current mesh to extract mesh from.");

				return active_volume->ExtractTriangleMesh();
					
					
		}

		std::shared_ptr<geometry::PointCloud>
			MovingTSDFVolume::ExtractCurrentVoxelPointCloud() {
				if (active_volume == NULL)
					utility::LogError("No current mesh to extract voxel point cloud from.");
				return active_volume->ExtractVoxelPointCloud();
		}

		std::vector<int> MovingTSDFVolume::get_kf_ids() {
			std::vector<int> t;
			for (auto mesh_unit : completed_meshes) {
				t.push_back(mesh_unit->keyframe_num);
			}
			return t;
		}

	}  // namespace integration
}  // namespace open3d
