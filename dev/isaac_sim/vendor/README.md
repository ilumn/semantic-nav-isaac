# Vendored runtime wheel

`clip-1.0-py3-none-any.whl` is built from the official OpenAI CLIP repository at
commit `d05afc436d78f1c48dc0dbf8e5980a9d471f35f6`. It retains OpenAI's MIT
license inside the wheel. The previous local wheel was removed because its
embedded AGPLv3 license did not match its documentation.

- Source: https://github.com/openai/CLIP
- SHA-256: `b0246e0be945d1ebc0711093683da47f39f9ce5c5ba5a27f9a30a142ba93f46e`

This local wheel prevents `bootstrap_runtime.sh --install-offline` from
attempting a Git fetch. Other Python wheels must still exist in the selected
`uv` cache for an offline install. Its import API and loading of the pinned
ViT-B/32 checkpoint were validated before inclusion.
