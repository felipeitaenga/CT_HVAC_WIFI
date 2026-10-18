#!/usr/bin/env python3
"""Empacota o firmware.bin para distribuição via OTA remoto (TJS/<id>/OTA/SET).

Copia o binário compilado para dist/hvac-<versao>.bin, calcula o SHA-256 e
imprime a linha pronta para publicar no tópico OTA/SET.

Uso automático: já referenciado em platformio.ini (extra_scripts = post:...) —
roda sozinho depois de `pio run`.

Uso manual: python scripts/publicar_ota.py   (depois de já ter rodado `pio run`)
"""
import hashlib
import re
import shutil
from pathlib import Path

URL_PREFIX = "https://bmstjs.pilaresautomacao.com.br/firmware/"


def ler_versao(root: Path) -> str:
    texto = (root / "src" / "main.cpp").read_text(encoding="utf-8")
    m = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', texto)
    if not m:
        raise RuntimeError("FIRMWARE_VERSION não encontrado em src/main.cpp")
    return m.group(1)


def empacotar(root: Path, bin_path: Path):
    versao = ler_versao(root)
    dist = root / "dist"
    dist.mkdir(exist_ok=True)
    destino = dist / f"hvac-{versao}.bin"
    shutil.copy2(bin_path, destino)

    sha256 = hashlib.sha256(destino.read_bytes()).hexdigest()
    url = URL_PREFIX + destino.name

    # Sem acentos/emoji: o console do Windows (cp1252) quebra ao imprimir alguns
    # caracteres durante o build do PlatformIO.
    print("\nOTA empacotado:", destino)
    print("   Tamanho :", destino.stat().st_size, "bytes")
    print("   SHA-256 :", sha256)
    print("\n   Comando para publicar em TJS/<id>/OTA/SET:")
    print(f"   {versao};{url};{sha256}\n")
    return destino, sha256, url


if __name__ == "__main__":
    # __file__ só existe numa execução normal do Python (não no exec() do SCons abaixo).
    root = Path(__file__).resolve().parent.parent
    bin_path = root / ".pio" / "build" / "esp32dev" / "firmware.bin"
    if not bin_path.exists():
        raise SystemExit(f"firmware.bin não encontrado em {bin_path} — rode 'pio run' primeiro")
    empacotar(root, bin_path)
else:
    # Executado pelo PlatformIO/SCons como extra_script (post-build) — Import() é
    # injetado pelo SCons nesse contexto e não existe numa execução manual.
    Import("env")  # noqa: F821

    def _pos_build(source, target, env):
        empacotar(Path(env.get("PROJECT_DIR")), Path(str(target[0])))

    env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", _pos_build)
