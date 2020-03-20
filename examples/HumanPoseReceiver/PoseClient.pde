import oscP5.*;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

final int KEY_POINT_COUNT = 18;

class PoseClient {
  private OscP5 osc;
  private List<Pose> poses;

  public PoseClient(int port) {
    poses = new CopyOnWriteArrayList<Pose>();
    osc = new OscP5(this, port);
  }

  private synchronized void oscEvent(OscMessage msg) {
    if (msg.checkAddrPattern("/poses")) {
      preparePoses(msg);
      return;
    }

    if (msg.checkAddrPattern("/pose")) {
      updatePose(msg);
      return;
    }
  }

  private void preparePoses(OscMessage msg) {
    int poseCount = msg.get(0).intValue();

    if (poseCount == poses.size())
      return;

    poses.clear();
    for (int i = 0; i < poseCount; i++) {
      poses.add(new Pose());
    }
  }

  private void updatePose(OscMessage msg) {
    int id = msg.get(0).intValue();
    float score = msg.get(1).floatValue();
    
    Pose pose = poses.get(id);
    pose.id = id;
    pose.score = score;
    
    for(int i = 0; i < KEY_POINT_COUNT; i++) {
      int index = 2 + (i * 2);
      pose.keypoints[i].x = msg.get(index).floatValue();
      pose.keypoints[i].y = msg.get(index + 1).floatValue();
    }
  }
}

class Pose {
  int id;
  float score;
  PVector[] keypoints;

  public Pose() {
    score = -1;
    keypoints = new PVector[KEY_POINT_COUNT];
    for (int i = 0; i < keypoints.length; i++) {
      keypoints[i] = new PVector();
    }
  }
}
