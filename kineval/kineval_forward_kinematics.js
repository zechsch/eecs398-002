
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () {

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();

}
kineval.buildFKTransforms = function buildFKTransforms(){

    //just build the base with the starting identity and base
    traverseFKBase(generate_identity(4), robot.base);
}

function traverseFKBase(xform, link){

    var trans, rotx,roty,rotz, rot;
    //generate translation and individual axis rotation matricies
    trans = generate_translation_matrix(robot.origin.xyz);
    rotx = generate_rotation_matrix_X(robot.origin.rpy[0]);
    roty = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    rotz = generate_rotation_matrix_Z(robot.origin.rpy[2]);

    //multiply to get overall rotation matrix
    rot = matrix_multiply(rotx, roty);
    rot = matrix_multiply(rot, rotz);



    //create xform from Identity*Translation*Rotation
    curXform = matrix_multiply(xform, trans);
    curXform = matrix_multiply(curXform, rot);

    robot_heading = matrix_multiply(curXform, matrix_from_vector([0,0,1,1]));
    robot_lateral = matrix_multiply(curXform, matrix_from_vector([1,0,0,1]));

    //set the xform for rendering
    robot.links[link].xform = curXform;

    //for keeping this xform around
    var baseXform = curXform;

    //loop through joints and if they are a child of base, iterate on them
    for(x in robot.joints){
        if(robot.joints[x].parent == robot.base){
            traverseFKJoint(baseXform, x);
        }
    }

}

function traverseFKJoint(xform, joint) {

    var trans, rotx,roty,rotz, rot, quatrot;

    //generate translation and individual axis rotation matricies
    trans = generate_translation_matrix(robot.joints[joint].origin.xyz);
    rotx = generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0]);
    roty = generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1]);
    rotz = generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]);
    rot = matrix_multiply(rotx, roty);
    rot = matrix_multiply(rot, rotz);

    quatrot = quaternion_from_axisangle(robot.joints[joint].axis, robot.joints[joint].angle);
    quatrot = quaternion_normalize(quatrot);
    quatrot = quaternion_to_rotation_matrix(quatrot);
    rot = matrix_multiply(rot, quatrot);
    //multiply by quaternion

    //create xform from Identity*Translation*Rotation
    curXform = matrix_multiply(xform, trans);
    curXform = matrix_multiply(curXform, rot);

    //set xform for rendering
    robot.joints[joint].xform = curXform;

    //if the child exists, iterate on it
    if(typeof robot.joints[joint].child != "undefined")
        traverseFKLink(curXform, robot.joints[joint].child);
}

function traverseFKLink(xform, link){

    //just set the xform to the link's parent's xform
    robot.links[link].xform = xform;

    //go through the link's children
    for( x in robot.joints){
        if (robot.joints[x].parent == link)
            traverseFKJoint(xform, x);
    }
}

    // STENCIL: reference code alternates recursive traversal over
    //   links and joints starting from base, using following functions:
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
