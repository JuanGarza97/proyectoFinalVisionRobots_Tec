#include <webots/Device.hpp>
#include <webots/Led.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>

#include <math.hpp>
#include <stdio.hpp>
#include <stdlib.hpp>

#define TIME_STEP 64

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  keyboard kb;

   camera *cm;
  cm=robot->getCamera("CAM");
  cm->enable(TIME_STEP);
  cm->getImage();
  
  
  ;