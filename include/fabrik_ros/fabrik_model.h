

#include <fabrik/base/fabrik.h>

#include <urdf/model.h>

/** \brief Convert urdf to fabrik robot model 
 * frames in urdf: http://wiki.ros.org/urdf/XML/joint
 * each link has a refernce frame in which all other frames are expressed, for example: joint frame, collision frame,
 * joint is attached to the child link and both(joint and link) have the same frame which is the reference frame
 * for the link. Collision frames, inertial frame and other properties of the child
 * link are expressed in this reference frame. The joint axis of the child link also is expressed in that.
 * Now this child's reference frame itself is expressed in the parent's reference frame.
 * 
 * Now, we want to covert these frames to fabrik frames
 * [panda_link1_s panda_link1_e] [] [] .... []
 * 
 * model->getRoot() gives the referecne frame of the fixed link of panda("panda_link0")
 * but what we want is the frame at the end of this link. Joint_1 is expressed in its parent which is panda_link0
 * so I need the origin property of joint_1 
 * 
 * 
 * Oooops, actually what I want is the relative transformation of child link with respect to parent.
 */ 

class FabrikModel
{
public:

FabrikModel(const urdf::Model& urdf_model);
FabrikModel(const std::string& urdf_file);



private:
urdf::Model urdf_model_;

};