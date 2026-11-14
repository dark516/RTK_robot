# Local LLM gateway

Сборка и актуальные команды: [REPRODUCIBILITY.md](../../../docs/REPRODUCIBILITY.md).

Gateway предоставляет три модели Qwen3.5 через один OpenAI-compatible API. В памяти находится выбранная модель; backend VLM на порту 8091 запускается отдельно. UI — `https://LAPTOP_IP:8082`, API — `http://LAPTOP_IP:8083/v1`, backend LLM — `127.0.0.1:8092`.

```bash
ros2 launch rover_vision local_llm_gateway.launch.py autostart:=false
```

Параметры `llama_server`, `models_dir`, `default_model`, `device` задаются launch-аргументами; `config_file` позволяет выбрать YAML для остальных параметров. Значения именованных launch-аргументов имеют приоритет над одноимёнными значениями YAML. Ключ — `~/.config/sverk-rover/local_llm_api_key`.

Идентификаторы: `qwen3.5-2b`, `qwen3.5-9b`, `qwen3.5-35b-a3b`. Загрузка модели по умолчанию в autostart=false отключена. Запрос инференса с ключом может загрузить выбранные веса, если они существуют; отсутствие весов не означает готовый backend.

В каталоге deploy сохранены исторические user-service шаблоны. Они не установлены автоматически при сборке этого репозитория и могут содержать прежние абсолютные пути.
