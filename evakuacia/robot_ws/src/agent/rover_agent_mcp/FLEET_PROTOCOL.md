# Fleet protocol integration

`rover_agent_mcp` 0.2 accepts both legacy plain text and the fleet JSON envelope on
`/agent/text_command`.

```json
{
  "message_id": "0b0d3c49-1404-4546-b31a-31897bbe7a7a",
  "robot_id": "rover-01",
  "text": "Проедь вперёд и сообщи результат"
}
```

Only `text` is passed to the LLM and MCP tools. The transport identifiers are kept
outside the prompt and automatically copied into `/agent/status` and
`/agent/answer`.

Status example:

```json
{
  "message_id": "0b0d3c49-1404-4546-b31a-31897bbe7a7a",
  "robot_id": "rover-01",
  "status": "running",
  "text": "Команда получена локальным агентом."
}
```

Final answer example:

```json
{
  "message_id": "0b0d3c49-1404-4546-b31a-31897bbe7a7a",
  "robot_id": "rover-01",
  "status": "completed",
  "text": "Готово."
}
```

The output `robot_id` is taken from the node parameter, not from the incoming
message. This prevents a malformed server payload from changing the agent's
identity. The incoming `message_id` is preserved for request/response correlation.
