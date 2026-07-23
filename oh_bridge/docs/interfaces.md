# Robot Task Interface


## LLM -> OpenHarmony


Format:

```json
{
  "schema_version":"robot_task.v1",
  "request_id":"task_001",
  "task":"pick_place",
  "source_object":"cup",
  "target_object":"bowl",
  "parameters":{
      "speed":0.25
  }
}
