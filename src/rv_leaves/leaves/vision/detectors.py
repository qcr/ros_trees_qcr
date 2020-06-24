from rv_trees.leaves_ros import ServiceLeaf

class GetFaces(ServiceLeaf):
  def __init__(self, name='Get face pose', service_name='/service/face_tracking', *args, **kwargs):
    super(GetFaces, self).__init__(
      name=name,
      service_name=service_name,
      save_fn=lambda leaf, value: self._default_save_fn(value.result if hasattr(value, 'result') else None),
      *args, 
      **kwargs
    )

class GetGraspPose(ServiceLeaf):
  def __init__(self, name='Get grasp pose', service_name='/service/ggcnn', *args, **kwargs):
    super(GetGraspPose, self).__init__(
      name=name,
      service_name=service_name,
      save_fn=lambda leaf, value: self._default_save_fn(value.result if hasattr(value, 'result') else None),
      *args, 
      **kwargs
    )

class GetYoloDetections(ServiceLeaf):
  def __init__(self, name="Get YOLO detections", service_name="/cloudvis/yolo", *args, **kwargs):
    super(GetYoloDetections, self).__init__(
      name=name, 
      service_name=service_name, 
      eval_fn=lambda leaf, value: hasattr(value, 'result') and len(value.result.detections) > 0 and all([d.class_label != '' for d in value.result.detections]),
      save_fn=lambda leaf, value: self._default_save_fn(value.result if hasattr(value, 'result') else None),
      *args, 
      **kwargs
    )

class GetValveDetections(ServiceLeaf):
  def __init__(self, name="Get Valve detections", service_name="/service/valves", *args, **kwargs):
    super(GetValveDetections, self).__init__(
      name=name, 
      service_name=service_name, 
      eval_fn=lambda leaf, value: hasattr(value, 'result') and len(value.result.detections) > 0 and all([d.class_label != '' for d in value.result.detections]),
      save_fn=lambda leaf, value: self._default_save_fn(value.result if hasattr(value, 'result') else None),
      *args, 
      **kwargs
    )

class GetHandDetections(ServiceLeaf):
  def __init__(self, name="Get hand detections", service_name="/service/hands", *args, **kwargs):
    super(GetHandDetections, self).__init__(
      name=name, 
      service_name=service_name, 
      eval_fn=lambda leaf, value: hasattr(value, 'result') and len(value.result.detections) > 0 and all([d.class_label != '' for d in value.result.detections]),
      save_fn=lambda leaf, value: self._default_save_fn(value.result if hasattr(value, 'result') else None),
      *args, 
      **kwargs
    )