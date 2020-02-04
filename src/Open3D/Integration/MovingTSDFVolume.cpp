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
			color_type_(color_type) {
				current_block = Eigen::Vector3i(0, 0, 0);
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
			const Eigen::Matrix4d &extrinsic) {
			if ((image.depth_.num_of_channels_ != 1) ||
				(image.depth_.bytes_per_channel_ != 4) ||
				(image.depth_.width_ != intrinsic.width_) ||
				(image.depth_.height_ != intrinsic.height_) ||
				(color_type_ == TSDFVolumeColorType::RGB8 &&
					image.color_.num_of_channels_ != 3) ||
					(color_type_ == TSDFVolumeColorType::RGB8 &&
						image.color_.bytes_per_channel_ != 1) ||
						(color_type_ == TSDFVolumeColorType::Gray32 &&
							image.color_.num_of_channels_ != 1) ||
							(color_type_ == TSDFVolumeColorType::Gray32 &&
								image.color_.bytes_per_channel_ != 4) ||
								(color_type_ != TSDFVolumeColorType::NoColor &&
									image.color_.width_ != intrinsic.width_) ||
									(color_type_ != TSDFVolumeColorType::NoColor &&
										image.color_.height_ != intrinsic.height_)) {
				utility::LogError(
					"[MovingTSDFVolume::Integrate] Unsupported image format.");
			}

			update_active_volume(extrinsic);

			active_volume->Integrate(image, intrinsic, extrinsic);

		}

		void MovingTSDFVolume::update_active_volume(Eigen::Matrix4d extrinsic) {
			std::lock_guard<std::mutex> guard(keyFrameLock);

			//printf("my point %f, %f, %f \n", extrinsic(0, 3), extrinsic(1, 3), extrinsic(2, 3));


			auto block_loc = LocateBlock(Eigen::Vector3d(extrinsic(0, 3), extrinsic(1, 3), extrinsic(2, 3)));

			//return if we are still in the same current_block
			if (block_loc.isApprox(current_block)) {
				return;
			}

			printf("EXITED BLOCK, NEW BLOCK\n");

			completed_meshes.push_back(active_volume->ExtractTriangleMesh());
			completed_meshes_transforms.push_back(active_volume_transform);
			completed_meshes_keyframe_nums.push_back(active_volume_keyframe_num);

			active_volume = new ScalableTSDFVolume(voxel_length_, sdf_trunc_, color_type_);
			active_volume_transform = latest_key_frame;
			active_volume_keyframe_num = latest_key_frame_num;

			current_block = block_loc;
		}



		void MovingTSDFVolume::set_latest_key_frame(Eigen::Matrix4d transform, int key_frame_num) {
			latest_key_frame = transform;
			latest_key_frame_num = key_frame_num;

			if (active_volume_keyframe_num == -1) {
				active_volume_keyframe_num = key_frame_num;
				active_volume_transform = transform;
			}
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

		//todo implement extracting from all the triangle meshes
		std::vector<std::shared_ptr<geometry::TriangleMesh>>
			MovingTSDFVolume::ExtractTriangleMeshes() {
				if (completed_meshes.size() == 0 && active_volume == NULL)
					utility::LogError("No meshes to extract mesh from.");

				std::vector<std::shared_ptr<geometry::TriangleMesh>> meshes;
				meshes.insert(meshes.end(), completed_meshes.begin(), completed_meshes.end());
				meshes.push_back(this->ExtractCurrentTriangleMesh());
				return meshes;
		}

		void MovingTSDFVolume::update_key_frames(std::map<int, Eigen::Matrix4d> keyframes) {
			std::lock_guard<std::mutex> guard(keyFrameLock);
			printf("loop closure detected -> key frames updating\n");

			for (int i = 0; i < completed_meshes.size(); i++) {
				Eigen::Matrix4d updated_transform = keyframes.find(completed_meshes_keyframe_nums[i])->second;

				//no changes
				if (updated_transform.isApprox(completed_meshes_transforms[i])) {
					continue;
				}
				else {
					Eigen::Matrix4d transform = updated_transform * completed_meshes_transforms[i].inverse();
					std::vector<Eigen::Vector3d> vertices = completed_meshes[i]->vertices_;
					for (int k = 0; k < vertices.size(); k++) {
						Eigen::Vector4d new_vertex(vertices[k](0), vertices[k](1), vertices[k](2), 1);
						new_vertex = transform * new_vertex;
						Eigen::Vector3d updated_vertex(new_vertex(0), new_vertex(1), new_vertex(2));
						vertices[k] = updated_vertex;
					}
					completed_meshes_transforms[i] = updated_transform;
				}

			}

		}

		std::vector<int> MovingTSDFVolume::get_kf_ids() {
			std::vector<int> t;
			t.insert(t.end(), completed_meshes_keyframe_nums.begin(), completed_meshes_keyframe_nums.end());
			t.push_back(active_volume_keyframe_num);
			return t;
		}

	}  // namespace integration
}  // namespace open3d
