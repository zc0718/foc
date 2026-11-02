from typing import TypeVar, Optional
from pathlib import Path
import numpy as np
import os
import json
import shutil
import re
import subprocess
sep = os.path.sep
Language = TypeVar('Language')
Version = TypeVar('Version')
_get_root_path_list = (lambda : (Path(__file__).__str__()).split(sep)[:-2])
_root_path_list = _get_root_path_list()
_pair_capture = {
    'h': 'c',
    'c': 'h',
    'hpp': 'cpp',
    'cpp': 'hpp'
}
lang_tag_map = {
    'en': 'en',
    'zh': 'zh_CN',
    'jp': 'ja'
}


def _get_root_path() -> str:
    return sep.join(_get_root_path_list())


def _inherit_root_metadata():
    with open(_get_root_path() + sep + 'metadata.json', 'r', encoding='utf-8') as f:
        _meta = json.load(f)
    return _meta


# The filters must accept all three bilingual shapes: compact ///< lines, indented blocks, continuation lines.
_hit_com_tag = re.compile(r"\s*\* @")
_hit_lang_tag = re.compile(rf"(\s*\* @[a-z]+).*\[({'|'.join(_inherit_root_metadata().get('doc_languages'))})] ")
_hit_free_tag = re.compile(r"\s*\* [^@]+")
_hit_file_start = re.compile(r"\n?/\*!")
_hit_since_start = re.compile(r"\n?/\*\*")
language_map = {'en': 'English', 'zh': 'Chinese', 'jp': 'Japanese'}

# Fixed filenames / annotations used in several places (extracted to avoid drift).
DOXYFILE_IN = 'Doxyfile.in'
SINCE_TAG = '@since '


def _match_file_doc(block: str):
    """Linear match: anchor the `/*!` start and locate @file/@defgroup via

    str.find to avoid backtracking-heavy regexes.
    """
    m = _hit_file_start.match(block)
    if m is None or '@file' not in block or '@defgroup' not in block:
        return None
    return m


def _match_since_command(block: str):
    """Linear match: anchor the `/**` start and locate SINCE_TAG via str.find

    to avoid backtracking-heavy regexes.
    """
    m = _hit_since_start.match(block)
    if m is None or SINCE_TAG not in block:
        return None
    return m


def _no_recursive_clean_img(x: str):
    if os.path.exists(x):
        for f in os.listdir(x):
            os.remove(x + sep + f)


def _idx_slicer(x: np.ndarray) -> list[list[int]]:
    res, _in_selection = [], False
    for i, v in enumerate(x):
        if not (v[0] == True and v[1] == True and v[2] == False):
            if v[0] == True and v[1] == False and v[2] == False:
                _in_selection = False
            if _in_selection:
                res[-1].append(i)
        else:
            res.append([i])
            _in_selection = True
    return res


def _determine_sub_groups(x: list[list[int]], languages: list[str],
                          _regex_ref: list[list[Optional[re.Match]]]) -> tuple[dict[str, list[int]], list[int]]:
    res = {k: [] for k in languages}
    for _set in x:
        _match = _regex_ref[_set[0]][1]
        res[_match.group(2)].extend(_set)
    return res, [_[0] for _ in x]


_lang_marker_re = re.compile(r"\[(en|zh|jp)\]\s?")
_tag_line_re = re.compile(r"^(\s*\*\s*)@([A-Za-z_][A-Za-z0-9_]*)(.*?)(\r?\n)?$")
_inline_doc_re = re.compile(r"^(.*?(?:///<?|\*\*<)\s*)(.*?)(\r?\n)?$")
_cjk_re = re.compile(r"[\u3000-\u303f\u4e00-\u9fff\uff01-\uff60]")


def _split_lang_segments(text: str):
    """Split 'a [en] b [zh] c' into its marked segments.

    Returns (lead, [(lang, text), ...]); lead is whatever precedes the first marker.
    """
    marks = list(_lang_marker_re.finditer(text))
    if not marks:
        return text, []
    _segs = []
    for _i, _m in enumerate(marks):
        _end = marks[_i + 1].start() if _i + 1 < len(marks) else len(text)
        _segs.append((_m.group(1), text[_m.end():_end]))
    return text[:marks[0].start()], _segs


def _rewrite_chunk(ch: dict) -> list[str]:
    """One tagged doc chunk -> one chunk per language, markers on the tag line."""
    _lead, _segs = _split_lang_segments(ch['rest'])
    _body_has_marker = any(_lang_marker_re.search(_l) for _l in ch['lines'])
    if not _segs or (len(_segs) == 1 and not _body_has_marker):
        return [ch['raw']]

    _primary = _segs[0][0]
    _order, _buckets = [], {}

    def _push(_lang: str, _text: str) -> None:
        if _lang not in _buckets:
            _order.append(_lang)
            _buckets[_lang] = []
        _text = _text.strip()
        if _text:
            _buckets[_lang].append(_text)

    for _lang, _text in _segs:
        _push(_lang, _text)

    for _line in ch['lines']:
        _content = _line.strip().lstrip('*').strip()
        if not _content:
            continue
        _c_lead, _c_segs = _split_lang_segments(_content)
        if _c_segs:
            for _lang, _text in _c_segs:
                _push(_lang, _text)
            continue
        # Body paragraphs carry no marker: CJK goes to the Chinese group, the rest to the opening language.
        _push('zh' if _cjk_re.search(_content) else _primary, _content)

    _indent = ch['prefix']
    _eol = ch['eol'] or '\n'
    # Text before the first marker is the documented name (@param/@tparam); dropping it renames the parameter.
    _head = (ch['tag'] + _lead).rstrip() + ' '
    _out = []
    _written = [k for k in _order if _buckets[k]]
    for _i, _lang in enumerate(_written):
        _items = _buckets[_lang]
        _out.append(f"{_indent}@{_head}[{_lang}] {_items[0]}{_eol}")
        for _extra in _items[1:]:
            _out.append(f"{_indent}       {_extra}\n")
        if _i + 1 < len(_written):
            _out.append(f"{_indent.rstrip()}\n")
    return _out


def _rewrite_block(block: list[str]) -> list[str]:
    """Rewrite one /** ... */ block into marker groups; untouched lines keep their bytes."""
    _chunks, _current = [], None
    for _line in block[1:-1]:
        _m = _tag_line_re.match(_line)
        if _m:
            _current = {'prefix': _m.group(1), 'tag': _m.group(2), 'rest': _m.group(3),
                        'eol': _m.group(4) or '', 'lines': [], 'raw': _line}
            _chunks.append(_current)
        elif _current is not None:
            _current['lines'].append(_line)
        else:
            _chunks.append({'prefix': None, 'tag': None, 'rest': None, 'eol': '',
                            'lines': [_line], 'raw': _line})

    _out = [block[0]]
    for _ch in _chunks:
        if _ch['tag'] is None:
            _out.extend(_ch['lines'])
        else:
            _out.extend(_rewrite_chunk(_ch))
    _out.append(block[-1])
    return _out


def _normalize_bilingual(lines: list[str]) -> list[str]:
    """Move every bilingual marker onto its own tag line, one group per language.

    Without this the filter downstream assigns a `[zh]` continuation line to the English group
    (English docs show Chinese) and drops it from the Chinese group (Chinese docs lose the text).
    """
    _out, _block, _in_block = [], [], False
    for _line in lines:
        _stripped = _line.lstrip()
        if not _in_block:
            if _stripped.startswith(('/**', '/*!')) and not _stripped.startswith('/**<') \
                    and '*/' not in _stripped[2:]:
                _in_block, _block = True, [_line]
                continue
            _out.append(_line)
            continue
        _block.append(_line)
        if '*/' in _line:
            _in_block = False
            _out.extend(_rewrite_block(_block) if any('[' in _b for _b in _block) else _block)
            _block = []
    if _block:
        _out.extend(_block)
    return _out


def _inline_lang_pick(line: str, lang_tag: Language) -> str:
    """Pick one language out of a compact one-line comment (`///`, `///<`, `/**<`)."""
    _m = _inline_doc_re.match(line)
    if _m is None:
        return line
    _lead, _segs = _split_lang_segments(_m.group(2))
    if len(_segs) < 2:
        return line
    for _lang, _text in _segs:
        if _lang == lang_tag:
            return f"{_m.group(1)}{_text.strip()}{_m.group(3) or ''}"
    return ''


def _language_filter(lines: list[str], langs: list[str], lang_tag: Language) -> list[str]:

    lines = _normalize_bilingual(lines)
    lines = [_inline_lang_pick(_l, lang_tag) for _l in lines]

    container = []
    _refs = [[_hit_com_tag.match(l), _hit_lang_tag.match(l), _hit_free_tag.match(l)] for l in lines]
    _refs_bool = np.array([[j is not None for j in i] for i in _refs])
    _idx_sets = _idx_slicer(_refs_bool)
    _lang_involved_lines, _lang_tag_hit_line = _determine_sub_groups(_idx_sets, langs, _refs)
    _all_lang_involved_lines = []

    for k, v in _lang_involved_lines.items():
        _all_lang_involved_lines.extend(v)

    for i, l in enumerate(lines):
        if i not in _all_lang_involved_lines:
            container.append(l)
        else:
            if i in _lang_involved_lines[lang_tag]:

                if i in _lang_tag_hit_line:
                    _sp = _refs[i][1].regs[2][1]
                    container.append(l[:(_sp-4)] + l[_sp + 1:])
                else:
                    container.append(l)

    return container


def _clean_doxygen_build(root: str):
    for f in os.listdir(root):
        full_path = root + sep + f
        if os.path.isdir(full_path):
            if f.startswith('_'):  # clean temporary folders
                shutil.rmtree(full_path)
            else:
                _clean_doxygen_build(full_path)
        elif f == DOXYFILE_IN:  # clean generated Doxyfile
            os.remove(full_path)


def _file_collector(folders: list[str], obj: list[str]) -> list[tuple[str, str]]:
    """Collect ``(folder_root, relative_path)`` for every suffix hit.

    Keeping the path relative to its configured folder is what lets the caller mirror the
    source layout instead of flattening it.
    """
    res = []
    for folder in folders:
        entry = Path(folder)
        for _obj in obj:
            for file in entry.rglob(f"*.{_obj}"):
                # relative paths stay POSIX internally: downstream compares them as strings
                # (tutorial location, Doxyfile injection, messages) and pathlib yields '\' here
                res.append((folder, str(file.relative_to(entry)).replace(sep, '/')))
    return res


def _walk_files(root: str) -> list[str]:
    """Relative paths of every file under ``root``."""
    res = []
    for _dir, _, _names in os.walk(root):
        for _name in _names:
            # same POSIX normalization as _file_collector: consumers compare these strings
            res.append(os.path.relpath(_dir + sep + _name, root).replace(sep, '/'))
    return res


def _mirror_roots(folders: list[str]) -> dict[str, str]:
    """Top-level name each configured doc folder gets inside the generated mirror.

    ``include`` -> ``include``, ``src`` -> ``src``, ``docs/doxygen/dox`` -> ``dox``. Two
    folders sharing a basename would silently collide, so that is a hard error.
    """
    res, _seen = {}, {}
    for folder in folders:
        _key = os.path.basename(folder.rstrip(sep))
        if _key in _seen:
            raise RuntimeError(f"doc_doxygen_folders entries '{_seen[_key]}' and '{folder}' "
                               f"both map to the mirror folder '{_key}'")
        _seen[_key] = folder
        res[folder] = _key
    return res


def _ver_should_include(x: str, ref_ver: str) -> bool:
    _x, _r = [int(_) for _ in x.split('.')], [int(_) for _ in ref_ver.split('.')]
    while len(_x) < 3:
        _x.append(0)
    while len(_r) < 3:
        _r.append(0)
    res = True  # for all equal
    for a, b in zip(_x, _r):
        if a < b:
            return True
        elif a > b:
            return False
        else:  # equal
            continue
    return res


def _ver_filter(x: list[str], ver: str) -> tuple[list[str], str]:
    x = (''.join(x)).split('\n\n\n')
    container, _need_append_end, file_ver = [], False, ''
    for i, block in enumerate(x):  # x[0] for import declaration
        if _match_file_doc(block):
            if SINCE_TAG in block:
                file_ver = block.split(SINCE_TAG)[1].split(' ')[0].strip()
            container.append(block+'\n//! @{')
            _need_append_end = True
        elif _match := _match_since_command(block):
            _obj_ver = block[_match.end():].split(SINCE_TAG)[1].split(' ')[0]  # version getter
            if _ver_should_include(_obj_ver, ver):
                container.append(block)
        else:
            container.append(block)
    if _need_append_end:
        container.append('//! @}\n')
    return container, file_ver


def _generate_docs_index(languages: list[str], versions: list[str], lib: str) -> str:
    lang_display = {
        'en': ('English', '🇬🇧'),
        'zh': ('中文', '🇨🇳'),
    }

    version_groups = ""
    for version in sorted(versions, reverse=True):
        group = f'    <!-- version {version} -->\n'
        group += '    <div class="version-group">\n'
        group += f'      <h2 class="version-title">Version {version}</h2>\n'
        group += '      <ul class="lang-links">\n'

        for lang in languages:
            lang_name, flag = lang_display.get(lang, (lang.capitalize(), ""))
            link_path = f"./{lang}/v{version}/build_sub/html/index.html"
            display_text = f"{flag} {lang_name} - Version {version}"
            group += '        <li>\n'
            group += f'          <a href="{link_path}">\n'
            group += f'            {display_text}\n'
            group += '          </a>\n'
            group += '        </li>\n'

        group += '      </ul>\n'
        group += '    </div>\n'
        version_groups += group

    html_content = f'''
    <!DOCTYPE html>
    <html lang="zh">
    <head>
      <meta charset="UTF-8" />
      <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
      <title>文档导航 - 多语言多版本</title>
      <style>
        body {{
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
          background-color: #f4f6f9;
          color: #333;
          margin: 0;
          padding: 40px;
        }}
        .container {{
          max-width: 800px;
          margin: auto;
          background-color: white;
          padding: 30px;
          border-radius: 12px;
          box-shadow: 0 6px 20px rgba(0, 0, 0, 0.1);
        }}
        h1 {{
          text-align: center;
          color: #2c3e50;
          margin-bottom: 30px;
          font-weight: 600;
        }}
        .version-group {{
          margin-bottom: 30px;
        }}
        .version-title {{
          font-size: 1.5em;
          color: #2980b9;
          border-bottom: 2px solid #3498db;
          padding-bottom: 8px;
          margin-bottom: 15px;
          display: inline-block;
        }}
        .lang-links {{
          list-style: none;
          padding: 0;
        }}
        .lang-links li {{
          margin: 12px 0;
        }}
        .lang-links a {{
          display: block;
          padding: 14px 20px;
          background-color: #ecf0f1;
          color: #2c3e50;
          text-decoration: none;
          border-radius: 8px;
          font-size: 1.1em;
          transition: all 0.3s ease;
          border-left: 4px solid #3498db;
        }}
        .lang-links a:hover {{
          background-color: #3498db;
          color: white;
          transform: translateX(6px);
          border-left-color: #f39c12;
        }}
        footer {{
          text-align: center;
          margin-top: 40px;
          color: #7f8c8d;
          font-size: 0.9em;
        }}
      </style>
    </head>
    <body>
      <div class="container">
        <h1>📄 {lib}项目文档导航</h1>
    
    {version_groups}
        <footer>
          &copy; 2025 {lib}项目文档中心. All rights reserved.
        </footer>
      </div>
    </body>
    </html>
    '''

    return html_content


def _capture_escape_files(x: list[str]) -> list[str]:
    """A version-dropped file drags its paired suffix along (``.h`` <-> ``.c`` etc.).

    splitext keeps a dotted directory name from confusing the pairing, and suffixes with
    no pair (``.dox``/``.cxx``) are skipped instead of inventing a name.
    """
    res = set(x)
    for _rel in x:
        _stem, _ext = os.path.splitext(_rel)
        _pair = _pair_capture.get(_ext.lstrip('.'))
        if _pair:
            res.add(f'{_stem}.{_pair}')
    return list(res)


class AutomationDoc:

    _root = _get_root_path()
    _images_source = sep.join(_root_path_list + ['docs', 'images'])
    _doxygen_root = sep.join(_root_path_list + ['docs', 'doxygen'])
    _images_doxygen_destination = sep.join(_root_path_list + ['docs', 'doxygen', 'images'])
    _images_sphinx_destination = sep.join(_root_path_list + ['docs', 'sphinx', 'images'])

    def __init__(self):
        self.meta = _inherit_root_metadata()
        self._copy_images_for_doxygen_and_sphinx()
        self.doxygen_automation()
        self.sphinx_automation()

    def doxygen_automation(self):
        self._validate_doc_sources()
        self._doxygen_scripts_from_sources_to_langs()
        self._doxygen_scripts_from_langs_to_vers()
        self._doxygen_config_injection()
        self._doxygen_config_execution()
        self._doxygen_export_navigation()
        self._doxygen_build_clean()

    def sphinx_automation(self):
        _path = sep.join(_root_path_list + ['docs', 'sphinx'])
        subprocess.run(["make", "-C", _path, "clean"])
        subprocess.run(["make", "-C", _path, "gettext"])
        _cmd = ["sphinx-intl", "update", "-p", _path + sep + "build" + sep + "gettext", "-d", _path + sep + "locales"]
        for _ in self.meta.get('doc_languages'):
            if _ != 'en':
                _cmd.append('-l')
                _cmd.append(lang_tag_map[_])
        subprocess.run(_cmd)
        for _ in self.meta.get('doc_languages'):
            if _ != 'en':
                subprocess.run(["make", "-C", _path, "-e", f"SPHINXOPTS=-D language={lang_tag_map[_]}", "html"])
        subprocess.run(["make", "-C", _path, "clean"])
        subprocess.run(["make", "-C", _path, "html"])
        # `make html` only renders the language conf.py pins, so emit one `build/html_<code>` per remaining language.
        _pinned = self._sphinx_pinned_language(_path)
        _sphinx = shutil.which('sphinx-build')
        if _sphinx is None:
            print('warning: sphinx-build is not on PATH; only `make html` output will be published')
            return
        for _lang in self.meta.get('doc_languages') or []:
            _tag = lang_tag_map.get(_lang, _lang)
            if _tag == _pinned:
                continue  # already emitted above as build/html
            _result = subprocess.run([_sphinx, "-q", "-b", "html", "-D", f"language={_tag}",
                                      _path + sep + "source",
                                      _path + sep + "build" + sep + f"html_{_lang}"])
            if _result.returncode != 0:
                # Never fatal: the published site is the pinned tree, so a broken translation must not fail the job.
                print(f'warning: sphinx build for language "{_tag}" failed '
                      f'(exit {_result.returncode}); build/html_{_lang} is incomplete')

    @staticmethod
    def _sphinx_pinned_language(_path: str) -> str:
        """The language `make html` renders, read from conf.py rather than assumed here."""
        _conf = Path(_path) / "source" / "conf.py"
        try:
            _match = re.search(r"^language\s*=\s*'([^']+)'", _conf.read_text(encoding='utf-8'), re.M)
        except OSError:
            return ''
        return _match.group(1) if _match else ''

    def _copy_images_for_doxygen_and_sphinx(self):
        # prefix syntax: IN: doxygen; OUT: sphinx; ALL: both

        # clean if exists
        _no_recursive_clean_img(self._images_doxygen_destination)
        _no_recursive_clean_img(self._images_sphinx_destination)

        # copy images to proper sub-doc systems
        for img in os.listdir(self._images_source):
            _f = self._images_source + sep + img
            if img.startswith('IN'):
                shutil.copy2(_f, self._images_doxygen_destination + sep + img)
            elif img.startswith('OUT'):
                shutil.copy2(_f, self._images_sphinx_destination + sep + img)
            else:  # both
                shutil.copy2(_f, self._images_doxygen_destination + sep + img)
                shutil.copy2(_f, self._images_sphinx_destination + sep + img)

    def _doxygen_scripts_from_sources_to_langs(self):

        # remove if exists
        _build_folder = self._doxygen_root + sep + 'build'
        if 'build' in os.listdir(self._doxygen_root):
            for _f in os.listdir(_build_folder):  # clean build folder (save isolated files for sub-git)
                _tmp = _build_folder + sep + _f
                if os.path.isdir(_tmp):
                    shutil.rmtree(_tmp)
        else:  # make folders
            os.mkdir(_build_folder)

        for _lang in self.meta.get('doc_languages'):
            _lang_folder = _build_folder + sep + _lang
            os.mkdir(_lang_folder)
            os.mkdir(_lang_folder + sep + f'_{_lang}_docstrings')
            for _ver in self.meta.get('doc_versions'):
                os.mkdir(_lang_folder + sep + f'v{_ver}')

        # move filtered docstring files, mirroring each source folder rather than flattening
        # it: the tree keeps include/ src/ dox/ apart, so the published docs get real
        # directory pages and two same-named files can no longer overwrite each other
        _folders = [self._root + sep + _ for _ in self.meta.get('doc_doxygen_folders')]
        _mirror = _mirror_roots(_folders)
        _files = _file_collector(_folders, self.meta.get('doc_doxygen_suffix'))
        for (_folder, _rel) in _files:
            with open(_folder + sep + _rel, 'r', encoding='utf-8') as f:
                _tmp = f.readlines()
            for _lang in self.meta.get('doc_languages'):
                _tmp_filtered = _language_filter(_tmp, self.meta.get('doc_languages'), _lang)
                _dst = (_build_folder + sep + _lang + sep + f'_{_lang}_docstrings'
                        + sep + _mirror[_folder] + sep + _rel)
                os.makedirs(os.path.dirname(_dst), exist_ok=True)
                with open(_dst, 'w', encoding='utf-8') as f:
                    f.write(''.join(_tmp_filtered))

    def _doxygen_scripts_from_langs_to_vers(self):

        _build_folder = self._doxygen_root + sep + 'build'
        for _lang in self.meta.get('doc_languages'):
            _f_out = _build_folder + sep + _lang
            for _ver in self.meta.get('doc_versions'):
                self._prepare_version_docstrings(_f_out, _lang, _ver)

    def _prepare_version_docstrings(self, _f_out, _lang, _ver):

        _f_in = _f_out + sep + f'v{_ver}'
        _docstrings = f'_{_lang}_v{_ver}_docstrings'
        _f_src = _f_out + sep + f'_{_lang}_docstrings'

        if _docstrings in os.listdir(_f_in):  # remove if exists
            shutil.rmtree(_f_in + sep + _docstrings)

        _f_final = _f_in + sep + _docstrings
        os.mkdir(_f_final)

        should_be_escape = self._filter_docstring_files(_f_src, _ver, _f_final)

        _escape_files = _capture_escape_files(should_be_escape)  # remove unmatched version files
        for _rel in _walk_files(_f_src):
            if _rel in _escape_files:
                os.remove(_f_final + sep + _rel)

    def _filter_docstring_files(self, _f_src, _ver, _f_final):

        should_be_escape = []
        for _rel in _walk_files(_f_src):
            with open(_f_src + sep + _rel, 'r', encoding='utf-8') as f:
                _tmp = f.readlines()

            _tmp, _file_ver = _ver_filter(_tmp, _ver)
            if _file_ver and not _ver_should_include(_file_ver, _ver):
                should_be_escape.append(_rel)

            _dst = _f_final + sep + _rel
            os.makedirs(os.path.dirname(_dst), exist_ok=True)
            with open(_dst, 'w', encoding='utf-8') as f:
                f.write('\n\n\n'.join(_tmp))
        return should_be_escape

    def _doxygen_config_injection(self):

        with open(self._root + sep + 'Doxyfile', 'r', encoding='utf-8') as f:
            _meta_config = ''.join(f.readlines())

        _meta_config = _meta_config.replace("%LIB_NAME%", self.meta.get('name'))
        _meta_config = _meta_config.replace("%PATTERNS%",
                                            ' '.join([f'*.{_}' for _ in self.meta.get('doc_doxygen_suffix')]))
        _meta_config = _meta_config.replace("%GRAPHVIZ_BIN%",
                                            self.meta.get('graphviz_bin') or '')   # absent = dot from PATH (downstream contract)

        _build_folder = self._doxygen_root + sep + 'build'
        for _lang in self.meta.get('doc_languages'):
            _f_out = _build_folder + sep + _lang
            for _ver in self.meta.get('doc_versions'):
                _f_in = _f_out + sep + f'v{_ver}'
                _meta = _meta_config.replace("%LAN%", _lang)
                _meta = _meta.replace("%VER%", _ver)
                _meta = _meta.replace("%FULL_LAN%", language_map.get(_lang))
                # @example only matches files sitting DIRECTLY under EXAMPLE_PATH (unlike
                # @include it ignores EXAMPLE_RECURSIVE), so the directories that actually
                # hold a tutorial have to be listed, plus the mirror root for @include
                _meta = _meta.replace("%EXAMPLE_DIRS%", ' '.join(
                    f"./_{_lang}_v{_ver}_docstrings/" + (f"{_}/" if _ else '')
                    for _ in self._example_dirs).strip())

                with open(_f_in + sep + DOXYFILE_IN, 'w', encoding='utf-8') as f:
                    f.write(_meta)

    def _doxygen_config_execution(self):

        _build_folder = self._doxygen_root + sep + 'build'
        for _lang in self.meta.get('doc_languages'):
            _f_out = _build_folder + sep + _lang
            for _ver in self.meta.get('doc_versions'):
                _leaf = _f_out + sep + f'v{_ver}'
                # check=True: a doxygen that dies must fail the build. It used to print
                # "successfully generated" whatever the exit status was.
                subprocess.run(["doxygen", DOXYFILE_IN], cwd=Path(_leaf), check=True)
                self._validate_doxygen_output(_leaf, _lang, _ver)
                print(f"Doxygen build system: Documentation of [{_lang}, v{_ver}] successfully generated")

    def _validate_doc_sources(self):
        """Completeness of the hand-written doc sources: landing page + tutorial.

        Deliberately asserts properties, not paths -- ``doc_doxygen_folders`` alone decides
        where documentation may live, so moving a file around inside the scanned tree stays
        green. Only a broken property fails.
        """
        _folders = [self._root + sep + _ for _ in self.meta.get('doc_doxygen_folders')]
        _files = _file_collector(_folders, self.meta.get('doc_doxygen_suffix'))
        _mirror = _mirror_roots(_folders)
        _rels = {_rel for _, _rel in _files}
        _names = {os.path.basename(_rel) for _rel in _rels}
        _problems, _mainpages, _tutorials, _sections = [], [], [], []

        for (_folder, _rel) in _files:
            if not _rel.lower().endswith('.dox'):
                continue
            with open(_folder + sep + _rel, 'r', encoding='utf-8') as f:
                _text = f.read()
            _here = _mirror[_folder] + '/' + _rel
            if '@mainpage' in _text:
                _mainpages.append(_here)
                _sections.extend(re.findall(r'@section\s+([A-Za-z0-9_]+)', _text))
            if '@example' in _text:
                _tutorials.append(_here)

        if not any(_rel.lower().endswith('.dox') for _rel in _rels):
            _problems.append('no .dox file under the scanned doc folders: check '
                             'doc_doxygen_folders covers where the docs live')
        if not _mainpages:
            _problems.append('no @mainpage found: the documentation has no landing page')
        elif len(_mainpages) > 1:
            _problems.append('more than one @mainpage (doxygen silently keeps whichever it '
                             'scans first): ' + ', '.join(sorted(_mainpages)))
        if not _tutorials:
            _problems.append('no @example page found: the documentation does not teach its own use')
        for _t in _tutorials:
            if '/demos/' not in f'/{_t}':
                _problems.append(f'{_t}: the tutorial belongs in demos/')

        for (_folder, _rel) in _files:
            with open(_folder + sep + _rel, 'r', encoding='utf-8') as f:
                _text = f.read()
            for _cmd in ('@include', '@example'):
                for _ref in re.findall(rf'{re.escape(_cmd)}\s+([^\s*]+)', _text):
                    if _ref not in _rels and os.path.basename(_ref) not in _names:
                        _problems.append(f'{_mirror[_folder]}/{_rel}: {_cmd} {_ref} does not '
                                         f'resolve inside the scanned tree')

        if _problems:
            raise RuntimeError('documentation completeness check failed:\n  - '
                               + '\n  - '.join(_problems))

        # what the generated leaves have to prove they consumed
        self._mainpage_sections = sorted(set(_sections))
        self._mainpage_pages = [f"{os.path.splitext(os.path.basename(_p))[0]}_8"
                                f"{os.path.splitext(os.path.basename(_p))[1].lstrip('.')}.html"
                                for _p in _mainpages]
        self._example_dirs = sorted({''} | {os.path.dirname(_t) for _t in _tutorials})

    def _validate_doxygen_output(self, _leaf, _lang, _ver):
        """Prove a generated leaf consumed both the landing page and the tutorial."""
        _html = _leaf + sep + 'build_sub' + sep + 'html'
        _problems = []
        _index = _html + sep + 'index.html'

        if not os.path.isfile(_index):
            _problems.append('no index.html')
        else:
            with open(_index, 'r', encoding='utf-8') as f:
                _text = f.read()
            if self._mainpage_sections:
                # @section ids survive the language and version filtering, so they mark the
                # landing page no matter which language or version is being built
                for _sec in self._mainpage_sections:
                    if _sec not in _text:
                        _problems.append(f'landing page lost mainpage section "{_sec}"')
            else:
                for _page in self._mainpage_pages:
                    if not os.path.isfile(_html + sep + _page):
                        _problems.append(f'mainpage source "{_page}" was not rendered')

        if not os.path.isdir(_html) or not [n for n in os.listdir(_html)
                                            if n.endswith('-example.html')]:
            _problems.append('no *-example.html: the tutorial page was not generated')

        if _problems:
            raise RuntimeError(f'doxygen output for [{_lang}, v{_ver}] is incomplete:\n  - '
                               + '\n  - '.join(_problems))

    def _doxygen_export_navigation(self):
        _tmp = _generate_docs_index(self.meta.get('doc_languages'), self.meta.get('doc_versions'),
                                    self.meta.get('name'))
        with open(self._doxygen_root + sep + 'build' + sep + 'docs.html', 'w', encoding='utf-8') as f:
            f.write(_tmp)

    def _doxygen_build_clean(self):
        _clean_doxygen_build(self._doxygen_root + sep + 'build')

    @staticmethod
    def _call_syntax_suggestion():
        _content = """
        ============================= Syntax Guide =============================
        1. one functional unit(single f.hpp without f.cpp, or a pair of f.hpp
           and f.cpp), share one namespace;
        2. multi-lined /*! ... */ for file-level docstring, @file followed by
           @defgroup command must be include (only once), with format 
           '@defgroup tag alias'; the alias must strictly match the namespace 
           of current functional unit;
        3. no need to include '@{' and '@}' for group domain, the build system
           can automatically process;
        4. three wrap lines to physically separate global object in file;
        5. multi-lined /** ... */ for global object level documenting, @since
           must be included when create;
        6. internalization support is available, use [lang] to note; no [lang]
           component is of global;
        7. images can be stored in ./docs/images, prefix 'IN_' for doxygen
           system, 'OUT_' for sphinx system, and 'ALL_' for both;
        8. ./docs/doxygen/dox/ for .dox files, syntax use multi-lined /*! ... */
           commands;
        9. ./docs/doxygen/dox/demos/ for demos, *.dox for tutorial files with
           pure /*! ... @example ... */ inside; and *.cxx for example files;
       10. .dox and .cxx are doc-only suffixes: they live only under
           ./docs/doxygen/dox/ and never under ./include/ or ./src/, which hold
           .h/.c/.hpp/.cpp only;
        ============================== Guide Over ==============================
        """
        print(_content)


if __name__ == '__main__':
    AutomationDoc()
