
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world

    // form configuration from base location and joint angles

    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0]) ||
    (q[0]>robot_boundary[1][0]) ||
    (q[2]<robot_boundary[0][2]) ||
    (q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    var trans, rotx,roty,rotz, rot, mstack = generate_identity(4);
    //generate translation and individual axis rotation matricies

    trans = generate_translation_matrix([q[0],q[1],q[2]]);
    rotx = generate_rotation_matrix_X(q[3]);
    roty = generate_rotation_matrix_Y(q[4]);
    rotz = generate_rotation_matrix_Z(q[5]);


    //multiply to get overall rotation matrix
    rot = matrix_multiply(rotx, roty);
    rot = matrix_multiply(rot, rotz);



    //create xform from Identity*Translation*Rotation
    mstack = matrix_multiply(mstack, trans);
    mstack = matrix_multiply(mstack, rot);
    return collision_FK_link(robot.base, mstack, q);

}

function collision_FK_link(link,mstack,q) {


  // this function is part of an FK recursion to test each link
  //   for collisions, along with a joint traversal function for
  //   the input robot configuration q
  //
  // this function returns the name of a robot link in collision
  //   or false if all its kinematic descendants are not in collision

  // test collision by transforming obstacles in world to link space
  mstack_inv = numeric.inv(mstack);
  // (alternatively) mstack_inv = matrix_invert_affine(mstack);
  link = robot.links[link];
  var i; var j;
  // test each obstacle against link bbox geometry
  //   by transforming obstacle into link frame and
  //   testing against axis aligned bounding box

  for (j in robot_obstacles) {


    var obstacle_local =
      matrix_multiply(mstack_inv,robot_obstacles[j].location);

    // assume link is in collision as default
    var in_collision = true;

    // return false if no collision is detected such that
    //   obstacle lies outside the link extents
    //   along any dimension of its bounding box
    //console.log(link.bbox);
    //console.log(j, 0);
    if ((obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius)) ||
      (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))){
      in_collision = false;
      //console.log(1);
  }

    if ((obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius)) ||
      (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))){
      in_collision = false;
      //console.log(2);
  }

    if ((obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius))||
      (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))){
      in_collision = false;
     // console.log(3);
  }

    // return name of link for detected collision if
    //   obstacle lies within the link extents
    //   along all dimensions of its bounding box
    if (in_collision)
      return link.name;

  }

  // recurse child joints for collisions,
  //   returning name of descendant link in collision
  //   or false if all descendants are not in collision
  if (typeof link.children !== 'undefined') {
    var local_collision;
    for (i in link.children) {
       // STUDENT: create this joint FK traversal function
       local_collision =
         collision_FK_joint(robot.joints[link.children[i]].name,
             mstack,q)
       if (local_collision)
         return local_collision;
     }
  }

  // return false, when no collision detected for this link and children
  return false;
}

function collision_FK_joint(joint, mstack, q){
    var trans, rotx,roty,rotz, rot, quatrot, mStack = mstack;

    //generate translation and individual axis rotation matricies
    trans = generate_translation_matrix(robot.joints[joint].origin.xyz);
    rotx = generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0]);
    roty = generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1]);
    rotz = generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]);
    rot = matrix_multiply(rotx, roty);
    rot = matrix_multiply(rot, rotz);

    //THIS FUCKING LINE
    quatrot = quaternion_from_axisangle(robot.joints[joint].axis, q[q_names[joint]]);

    quatrot = quaternion_normalize(quatrot);
    quatrot = quaternion_to_rotation_matrix(quatrot);
    rot = matrix_multiply(rot, quatrot);
    //multiply by quaternion

    //create xform from Identity*Translation*Rotation
    mStack = matrix_multiply(mStack, trans);
    mStack = matrix_multiply(mStack, rot);

    //if the child exists, iterate on it
    if(typeof robot.joints[joint].child != "undefined")
        return collision_FK_link(robot.joints[joint].child, mStack, q);
    /*if(typeof robot.joints[joint].child != "undefined"){

        return collision_FK_link(robot.joints[joint].child,
            robot.links[robot.joints[joint].child].xform, q);
    }
    else return false;*/
}
