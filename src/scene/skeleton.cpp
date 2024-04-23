#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const
{
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	// y axis points in the direction of extent:
	y = extent.unit();
	// if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid())
	{
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	// x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x, y) * y).unit();
	if (!x.valid())
	{
		// if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x, y) * y).unit(); //(this should do nothing)
	}

	// z computed from x,y:
	z = cross(x, y);

	// x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector<Mat4> Skeleton::bind_pose() const
{
	// A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector<Mat4> bind;
	bind.reserve(bones.size());
	if (!bones.size())
		return {};
	// NOTE: bones is guaranteed to be ordered such that parents appear before child bones.
	for (auto const &bone : bones)
	{
		//(void)bone; //avoid complaints about unused bone
		// placeholder -- your code should actually compute the correct transform:
		if (bone.parent == -1U)
		{
			bind.emplace_back(Mat4::translate(base));
			continue;
		}
		Mat4 bind_pose = bind[bone.parent] * Mat4::translate(bones[bone.parent].extent);
		bind.emplace_back(bind_pose);
	}

	assert(bind.size() == bones.size()); // should have a transform for every bone.
	return bind;
}

std::vector<Mat4> Skeleton::current_pose() const
{
	// A4T2a: bone-to-skeleton transformations in the current pose

	// Similar to bind_pose(), but takes rotation from Bone::pose into account.
	//  (and translation from Skeleton::base_offset!)
	std::vector<Mat4> pose;
	pose.reserve(bones.size());
	if (!bones.size())
		return {};
	// You'll probably want to write a loop similar to bind_pose().
	for (auto const &bone : bones)
	{
		// Useful functions:
		// Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
		// Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.
		Vec3 x, y, z;
		bone.compute_rotation_axes(&x, &y, &z);
		// std::cout << "x_axis:" << x << "\n";
		// std::cout << "y_axis:" << y << "\n";
		// std::cout << "z_axis:" << z << "\n";
		Mat4 R_zyx = Mat4::angle_axis(bone.pose.z, z) * Mat4::angle_axis(bone.pose.y, y) * Mat4::angle_axis(bone.pose.x, x);
		// std::cout << Mat4::angle_axis(bone.pose.z, z)[0] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.z, z)[1] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.z, z)[2] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.z, z)[3] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.y, y)[0] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.y, y)[1] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.y, y)[2] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.y, y)[3] << "\n\n";
		// std::cout << Mat4::angle_axis(bone.pose.x, x)[0] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.x, x)[1] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.x, x)[2] << "\n";
		// std::cout << Mat4::angle_axis(bone.pose.x, x)[3] << "\n";
		if (bone.parent == -1U)
		{
			pose.emplace_back(Mat4::translate(base + base_offset) * R_zyx);
			continue;
		}
		Mat4 T = Mat4::translate(bones[bone.parent].extent);
		pose.emplace_back(pose[bone.parent] * T * R_zyx);
		// std::cout << bind.back() << "\n";
	}
	assert(pose.size() == bones.size()); // should have a transform for every bone.
	return pose;
}

std::vector<Vec3> Skeleton::gradient_in_current_pose() const
{
	// A4T2b: IK gradient

	// Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	// The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector<Vec3> gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});

	// TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	// remember bone.compute_rotation_axes() -- should be useful here, too!
	for (auto &handle : handles)
	{
		if (!handle.enabled)
			continue;
		Vec3 h = handle.target;
		auto pose = current_pose();
		Vec4 p_4d = Mat4::I * pose.at(handle.bone) * Mat4::translate(bones[handle.bone].extent) * Vec4{0.0f, 0.0f, 0.0f, 1.f};
		Vec3 p = p_4d.xyz();

		for (BoneIndex b = handle.bone; b < bones.size(); b = bones[b].parent)
		{
			if (b == -1U)
				break;

			Bone const &bone = bones[b];
			Mat4 xf;
			if (bone.parent != -1U)
				xf = pose.at(bone.parent) * Mat4::translate(bones[bone.parent].extent);
			else
				xf = Mat4::translate(base + base_offset);

			Vec3 r = xf * Vec3(0.f, 0.f, 0.f);
			Vec3 x_axis, y_axis, z_axis;
			bone.compute_rotation_axes(&x_axis, &y_axis, &z_axis);

			Vec3 x = (xf * Mat4::angle_axis(bone.pose.z, z_axis) * Mat4::angle_axis(bone.pose.y, y_axis)).rotate(x_axis).unit();
			Vec3 y = (xf * Mat4::angle_axis(bone.pose.z, z_axis)).rotate(y_axis).unit(); //????
			Vec3 z = xf.rotate(z_axis).unit();
			// std::cout << dot(cross(x, p - r), p - h) << ", " << dot(cross(y, p - r), p - h) << ", " << dot(cross(z, p - r), p - h) << "\n";
			gradient[b].x += dot(cross(x, p - r), p - h);
			gradient[b].y += dot(cross(y, p - r), p - h);
			gradient[b].z += dot(cross(z, p - r), p - h);
		}
	}

	assert(gradient.size() == bones.size());
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps)
{
	// A4T2b - gradient descent
	// check which handles are enabled
	// run `steps` iterations
	std::vector<Vec3> gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});
	float gradient_norm = 0.f;
	while (steps)
	{
		steps--;
		gradient_norm = 0.f;
		// call gradient_in_current_pose() to compute d loss / d pose
		// add ...
		gradient = gradient_in_current_pose();
		for (size_t i = 0; i < bones.size(); i++)
		{
			gradient_norm += gradient[i].norm();
		}
		// std::cout << gradient_norm << "\n";
		//  if at a local minimum (e.g., gradient is near-zero), return 'true'.
		//  if run through all steps, return `false`.
		if (gradient_norm < 0.001)
			return true;

		for (size_t i = 0; i < bones.size(); i++)
		{
			bones[i].pose -= gradient[i];
		}
	}
	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p)
{
	// A4T3: bone weight computation (closest point helper)

	// Return the closest point to 'p' on the line segment from a to b

	// Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)
	float t;
	t = dot(p - a, b - a) / dot(b - a, b - a);
	if (t < 0)
		return a;
	else if (t > 1)
		return b;
	else
		return a + t * (b - a);
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const
{
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; // avoid complaints about unused mesh

	// A4T3: bone weight computation

	// visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)
	// be sure to use bone positions in the bind pose (not the current pose!)
	auto bind = bind_pose();
	for (auto &v : mesh.vertices)
	{
		Vec3 a, b, p, p_pi;
		float w, d, total_weight = 0.f;
		std::vector<Halfedge_Mesh::Vertex::Bone_Weight> new_bone_weights;
		for (size_t i = 0; i < bones.size(); i++)
		{
			a = (bind.at(i) * Vec4{0.0f, 0.0f, 0.0f, 1.f}).xyz();
			b = (bind.at(i) * Mat4::translate(bones[i].extent) * Vec4{0.0f, 0.0f, 0.0f, 1.f}).xyz();
			// p = bind.at(i) * Vec4{Vec3(v.position), 1.f}.xyz();
			p = v.position;
			p_pi = closest_point_on_line_segment(a, b, p);
			d = (p - p_pi).norm();

			// a = Vec3{0, 0, 0};
			// b = bones[i].extent;
			// p = v.position;
			// p_pi = closest_point_on_line_segment(a, b, p);
			// d = (p - p_pi).norm();
			w = std::max(0.f, bones[i].radius - d) / bones[i].radius;
			if (w > 0)
			{
				total_weight += w;
				Halfedge_Mesh::Vertex::Bone_Weight bw;
				bw.bone = (uint32_t)i;
				bw.weight = w;
				new_bone_weights.emplace_back(bw);
			}
		}

		if (total_weight > 0.f)
		{
			// normalization
			for (auto &bone_weight : new_bone_weights)
			{
				bone_weight.weight /= total_weight;
			}
			// assign weights
			v.bone_weights = new_bone_weights;
		}
	}

	// you should fill in the helper closest_point_on_line_segment() before working on this function
}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector<Mat4> const &bind, std::vector<Mat4> const &current)
{
	assert(bind.size() == current.size());

	// one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	//  then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map<Halfedge_Mesh::VertexCRef, Vec3> skinned_positions;
	std::unordered_map<Halfedge_Mesh::HalfedgeCRef, Vec3> skinned_normals;
	// reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)
	std::vector<Mat4> bind_inv = bind;
	for (Mat4 &b_inv : bind_inv)
	{
		b_inv = Mat4::inverse(b_inv);
	}
	std::vector<Mat4> trans;
	trans.reserve(bind.size());
	for (size_t i = 0; i < bind.size(); i++)
	{
		trans.emplace_back(current[i] * bind_inv[i]);
	}

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi)
	{
		Vec3 v_pos(0.f, 0.f, 0.f);
		Mat4 weight_mat = 0 * Mat4::I;
		for (auto &bw : vi->bone_weights)
		{
			weight_mat += bw.weight * trans[bw.bone];
		}
		v_pos = (weight_mat * Vec4{vi->position, 1.f}).xyz();
		// NOTE: vertices with empty bone_weights should remain in place.
		if (vi->bone_weights.empty())
		{
			skinned_positions.emplace(vi, vi->position);
			weight_mat = Mat4::I;
		}
		else
			skinned_positions.emplace(vi, v_pos); // PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.

		// circulate corners at this vertex:
		auto h = vi->halfedge;
		do
		{
			// NOTE: could skip if h->face->boundary, since such corners don't get emitted
			if (!h->face->boundary)
			{
				Vec3 trans_normal(0.f, 0.f, 0.f);
				trans_normal = (Mat4::transpose(Mat4::inverse(weight_mat)) * Vec4{h->corner_normal, 1.f}).xyz().unit();
				// trans_normal = weight_mat.inverse().T().rotate(h->corner_normal).unit();
				skinned_normals.emplace(h, trans_normal); // PLACEHOLDER! Replace with code that properly transforms the normal vector! Make sure that you normalize correctly.
			}

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	// Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	// Indexed_Mesh result = Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges); // PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.
	std::vector<Indexed_Mesh::Vert> verts;
	std::vector<Indexed_Mesh::Index> idxs;
	for (Halfedge_Mesh::FaceCRef f = mesh.faces.begin(); f != mesh.faces.end(); f++)
	{
		if (f->boundary)
			continue;

		// every corner gets its own copy of a vertex:
		uint32_t corners_begin = static_cast<uint32_t>(verts.size());
		Halfedge_Mesh::HalfedgeCRef h = f->halfedge;
		do
		{
			Indexed_Mesh::Vert vert;
			vert.pos = skinned_positions[h->vertex];
			vert.norm = skinned_normals[h];
			vert.uv = h->corner_uv;
			vert.id = f->id;
			verts.emplace_back(vert);
			h = h->next;
		} while (h != f->halfedge);
		uint32_t corners_end = static_cast<uint32_t>(verts.size());

		// divide face into a triangle fan:
		for (size_t i = corners_begin + 1; i + 1 < corners_end; i++)
		{
			idxs.emplace_back(corners_begin);
			idxs.emplace_back(static_cast<uint32_t>(i));
			idxs.emplace_back(static_cast<uint32_t>(i + 1));
		}
	}
	Indexed_Mesh result = Indexed_Mesh(std::move(verts), std::move(idxs));
	return result;
}

void Skeleton::for_bones(const std::function<void(Bone &)> &f)
{
	for (auto &bone : bones)
	{
		f(bone);
	}
}

void Skeleton::erase_bone(BoneIndex bone)
{
	assert(bone < bones.size());
	// update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b)
	{
		if (bones[b].parent == -1U)
			continue;
		if (bones[b].parent == bone)
		{
			assert(b > bone); // topological sort!
			// keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		}
		else if (bones[b].parent > bone)
		{
			assert(b > bones[b].parent); // topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	// update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */)
	{
		if (handles[h].bone == bone)
		{
			erase_handle(h);
		}
		else if (handles[h].bone > bone)
		{
			handles[h].bone -= 1;
			++h;
		}
		else
		{
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle)
{
	assert(handle < handles.size());

	// nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}

Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent)
{
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	// all other parameters left as default.

	// slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set<uint32_t> used;
	for (auto const &b : bones)
	{
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id))
		++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	// all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target)
{
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	// all other parameters left as default.

	// slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set<uint32_t> used;
	for (auto const &h : handles)
	{
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id))
		++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}

Skeleton Skeleton::copy()
{
	// turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid()
{
	for (uint32_t b = 0; b < bones.size(); ++b)
	{
		if (!(bones[b].parent == -1U || bones[b].parent < b))
		{
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty())
	{
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h)
	{
		if (handles[h].bone >= HandleIndex(bones.size()))
		{
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const
{
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const
{
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy()
{
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
