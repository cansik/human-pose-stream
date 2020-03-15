PoseClient client;

void setup() {
  size(640, 360, FX2D);

  client = new PoseClient(7400);
}

void draw() {
  background(0);

  // draw pose points
  for (Pose pose : client.poses) {
    for(int i = 0; i < pose.keypoints.length; i++) {
      PVector keypoint = pose.keypoints[i];
      circle(keypoint.x * width, keypoint.y * height, 10);
    }
  }
}
