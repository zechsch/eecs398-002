
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
    return robot_collision_forward_kinematics(q);

}

//function collision_FK_link(link,mstack,q) {

  // this function is part of an FK recursion to test each link
  //   for collisions, along with a joint traversal function for
  //   the input robot configuration q
  //
  // this function returns the name of a robot link in collision
  //   or false if all its kinematic descendants are not in collision

  // test collision by transforming obstacles in world to link space
  //mstack_inv = numeric.inv(mstack);
  // (alternatively) mstack_inv = matrix_invert_affine(mstack);
function robot_collision_forward_kinematics(q){

    return collision_FK_link(robot.base, robot.links["base"].xform, q);
    /*for(x in robot.joints){
        if(robot.joints[x].parent == robot.base){
            return collision_FK_joint(x, robot.joints[x].xform, q);
        }
    }*/
}
