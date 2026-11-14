from rover_agent_mcp.agent_text_node import AgentTextNode


def test_local_api_key_file_takes_priority_over_environment(tmp_path, monkeypatch):
    key_file = tmp_path / 'local_llm_api_key'
    key_file.write_text('key-from-file\n', encoding='utf-8')
    monkeypatch.setenv('OPENAI_API_KEY', 'key-from-environment')

    value = AgentTextNode._resolve_api_key(
        str(key_file),
        ('OPENAI_API_KEY',),
    )

    assert value == 'key-from-file'


def test_missing_key_file_falls_back_to_environment(tmp_path, monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY', 'fallback-key')

    value = AgentTextNode._resolve_api_key(
        str(tmp_path / 'missing'),
        ('OPENAI_API_KEY',),
    )

    assert value == 'fallback-key'
