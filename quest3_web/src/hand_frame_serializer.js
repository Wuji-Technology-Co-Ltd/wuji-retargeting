export const WEBXR_HAND_JOINTS = [
  "wrist",
  "thumb-metacarpal",
  "thumb-phalanx-proximal",
  "thumb-phalanx-distal",
  "thumb-tip",
  "index-finger-metacarpal",
  "index-finger-phalanx-proximal",
  "index-finger-phalanx-intermediate",
  "index-finger-phalanx-distal",
  "index-finger-tip",
  "middle-finger-metacarpal",
  "middle-finger-phalanx-proximal",
  "middle-finger-phalanx-intermediate",
  "middle-finger-phalanx-distal",
  "middle-finger-tip",
  "ring-finger-metacarpal",
  "ring-finger-phalanx-proximal",
  "ring-finger-phalanx-intermediate",
  "ring-finger-phalanx-distal",
  "ring-finger-tip",
  "pinky-finger-metacarpal",
  "pinky-finger-phalanx-proximal",
  "pinky-finger-phalanx-intermediate",
  "pinky-finger-phalanx-distal",
  "pinky-finger-tip",
];

export function emptyHand() {
  return { valid: false, joint_names: [], positions: [], orientations_xyzw: [] };
}

export function emptyArmWrist() {
  return { valid: false, position: [0, 0, 0], orientation_xyzw: [0, 0, 0, 1] };
}

export function serializeArmWrist(inputSource, frame, referenceSpace) {
  if (!inputSource || !inputSource.hand) return emptyArmWrist();
  const joint = inputSource.hand.get("wrist");
  const pose = joint ? frame.getJointPose(joint, referenceSpace) : null;
  if (!pose) return emptyArmWrist();
  return {
    valid: true,
    position: [pose.transform.position.x, pose.transform.position.y, pose.transform.position.z],
    orientation_xyzw: [
      pose.transform.orientation.x,
      pose.transform.orientation.y,
      pose.transform.orientation.z,
      pose.transform.orientation.w,
    ],
  };
}

export function serializeHand(inputSource, frame, referenceSpace) {
  if (!inputSource || !inputSource.hand) return emptyHand();
  const positions = [];
  const orientations = [];
  for (const name of WEBXR_HAND_JOINTS) {
    const joint = inputSource.hand.get(name);
    const pose = joint ? frame.getJointPose(joint, referenceSpace) : null;
    if (!pose) return emptyHand();
    positions.push([pose.transform.position.x, pose.transform.position.y, pose.transform.position.z]);
    orientations.push([
      pose.transform.orientation.x,
      pose.transform.orientation.y,
      pose.transform.orientation.z,
      pose.transform.orientation.w,
    ]);
  }
  return { valid: true, joint_names: WEBXR_HAND_JOINTS, positions, orientations_xyzw: orientations };
}
