# Sphinx Domain Guide（Sphinx 领域指南）

> Loaded on demand when the task involves Sphinx pages / translations. 涉及 Sphinx 页面/翻译时按需加载。
> The Sphinx side is the *thin* half of the docs pipeline — this guide fills in the RST authoring + i18n workflow. Sphinx 侧是文档流水线中偏弱的一半，本指南补全 RST 编写与翻译工作流。

## Directory（目录结构）

```
docs/sphinx/
├── Makefile / make.bat          # standard Sphinx make targets（标准 Sphinx 命令）
├── source/
│   ├── conf.py                  # Sphinx config（语言/locale/主题/多版本）
│   ├── index.rst                # landing page + toctree（入口 + 目录树）
│   ├── export.rst               # user-concern page
│   ├── components/              # per-component pages: cmake.rst / conan.rst / meta.rst
│   └── _static/                 # custom.css etc.
└── locales/
    └── zh_CN/LC_MESSAGES/       # translations: index.po, export.po, components/*.po (+ compiled .mo)
```

## conf.py Key Settings（conf.py 关键配置）

- `languages` / `language`: from metadata `doc_languages` via `lang_tag_map`（en→en, zh→zh_CN, jp→ja）; default `zh_CN`. 语言来自 metadata，zh 映射到 zh_CN 目录。
- `locale_dirs = ['../locales/']`, `gettext_compact = False` → **one .po per RST file**（每个 RST 一个独立 .po）.
- `html_theme = 'sphinx_rtd_theme'`; `html_context['versions']` = multi-version switcher（多版本切换器）.
- `version` auto-allocated to the nearest `doc_versions` entry (see `_allocate_doc_version`). 版本自动就近分配。

## Authoring RST Pages（编写 RST 页面）

1. **Register in the toctree**: add the new page to `index.rst` (or a component page), otherwise it is not built. 新页面必须挂进 toctree，否则不构建。
2. **i18n-mark strings** for translation — wrap with `` _` `` + backtick + backtick（用 `_` + 反引号包裹可翻译文本）:
   ```rst
   _`Metadata Configuration`
   =========================

   _`Key Settings`
   ---------------
   ```
   Backtick-marked strings become gettext messages（反引号标记的文本进入 .po 成为可翻译字符串）.
3. Images: `docs/images/` with `OUT:` prefix for sphinx (see `docs/build.py`). Sphinx 用 OUT: 前缀图片。

## Translation Workflow（翻译工作流）

1. **Extract new strings（提取新字符串）**:
   ```bash
   cd docs/sphinx
   make gettext                                    # -> build/gettext/*.pot
   sphinx-intl update -p build/gettext -d locales  # update locales/*/LC_MESSAGES/*.po
   ```
   (`docs/build.py` runs the `sphinx-intl update` step automatically.) build.py 自动做 update。
2. **Translate（翻译）**: edit `locales/<lang>/LC_MESSAGES/<file>.po` — fill `msgstr` for each `msgid`. 填 msgstr；中文在 `zh_CN/LC_MESSAGES/`。
3. **Compile .mo（编译 .mo，必须）**:
   ```bash
   sphinx-intl build -d locales   # compile .po -> .mo（.mo 不存在时页面仍显示旧翻译）
   ```
4. **Build per language（按语言构建）**:
   ```bash
   make -C docs/sphinx -e SPHINXOPTS="-D language=zh_CN" html
   ```
   (`docs/build.py` loops all `doc_languages`.) build.py 遍历所有语言。

## Pitfalls（易踩坑）

1. New RST not in toctree → silently not built. 新页面没挂 toctree 不会构建。
2. New strings extracted but `.po` not translated / `.mo` not compiled → stale translation shows. 提取了但没翻译或没编译 .mo，页面仍是旧翻译。
3. `gettext_compact = False` → each file has its own .po; fill the right file. 每个文件独立 .po，别填错文件。
4. Language `zh` maps to folder `zh_CN` (`lang_tag_map`); wrong folder = no translation. zh→zh_CN 目录映射，放错目录不生效。
5. Iterate with `python ./docs/build.py` locally. 本地迭代更快。

