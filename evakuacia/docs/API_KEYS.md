# API-ключи

Реальные ключи в репозитории не хранятся. Один bearer token используется для
OpenAI-compatible LLM gateway на ноутбуке и передаётся роботу через файл с
правами `0600` либо переменную `OPENAI_API_KEY`.

Рекомендуемые пути:

- ноутбук: `~/.config/sverk-rover/local_llm_api_key`;
- робот: `~/.config/sverk-rover/local_llm_api_key` (домашний каталог пользователя, запускающего узел).

Файлы ключей, TLS private keys, GGUF-веса и runtime-логи исключены через
`.gitignore`.
