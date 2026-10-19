#!/usr/bin/env python3
"""Empacota o firmware.bin para distribuição via OTA remoto (TJS/<id>/OTA/SET).

Lê TIPO_CONTROLADORA, MODELO_CONTROLADORA e FIRMWARE_VERSION do src/main.cpp,
copia o binário compilado para dist/<modelo minúsculo>-<versao>.bin
(ex.: fc300-1.4.3.bin), confere a assinatura NXFW dentro do .bin, calcula
MD5 (o que o firmware valida) e SHA-256, e imprime a linha pronta para
publicar no tópico OTA/SET. Padrão: docs/PADRAO_CONTROLADORAS.md

Uso automático: já referenciado em platformio.ini (extra_scripts = post:...) —
roda sozinho depois de `pio run`.

Uso manual: python scripts/publicar_ota.py   (depois de já ter rodado `pio run`)
"""
import hashlib
import re
import shutil
from pathlib import Path

# O NX-360 hospeda o arquivo em <URL_PREFIX><token>/<arquivo>.bin
URL_PREFIX = "https://bmstjs.pilaresautomacao.com.br/api/firmware/files/"


def ler_define(texto: str, nome: str) -> str:
    m = re.search(r'#define\s+' + nome + r'\s+"([^"]+)"', texto)
    if not m:
        raise RuntimeError(f"{nome} não encontrado em src/main.cpp")
    return m.group(1)


def ler_identificacao(root: Path):
    texto = (root / "src" / "main.cpp").read_text(encoding="utf-8")
    return (ler_define(texto, "TIPO_CONTROLADORA"),
            ler_define(texto, "MODELO_CONTROLADORA"),
            ler_define(texto, "FIRMWARE_VERSION"))


def empacotar(root: Path, bin_path: Path):
    tipo, modelo, versao = ler_identificacao(root)
    dados = bin_path.read_bytes()

    # Sem acentos/emoji: o console do Windows (cp1252) quebra ao imprimir alguns
    # caracteres durante o build do PlatformIO.
    assinatura = f"NXFW|TIPO={tipo}|MODELO={modelo}|VERSAO={versao}|"
    if assinatura.encode("ascii") not in dados:
        raise RuntimeError(f"Assinatura '{assinatura}' NAO encontrada em {bin_path}")

    dist = root / "dist"
    dist.mkdir(exist_ok=True)
    destino = dist / f"{modelo.lower()}-{versao}.bin"
    shutil.copy2(bin_path, destino)

    md5 = hashlib.md5(dados).hexdigest()
    sha256 = hashlib.sha256(dados).hexdigest()
    url = URL_PREFIX + "<token>/" + destino.name

    print("\nOTA empacotado:", destino)
    print("   Tipo      :", tipo)
    print("   Modelo    :", modelo)
    print("   Versao    :", versao)
    print("   Assinatura:", assinatura, "(OK, presente no .bin)")
    print("   Tamanho   :", len(dados), "bytes")
    print("   MD5       :", md5, "(validado pelo firmware)")
    print("   SHA-256   :", sha256)
    print("\n   Comando para publicar em TJS/<id>/OTA/SET (<token> = gerado pelo NX-360):")
    print(f"   {versao};{url};{md5}\n")
    return destino, md5, sha256


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
