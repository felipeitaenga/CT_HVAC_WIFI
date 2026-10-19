# Padrão comum das controladoras TJS / NX-360

Contrato que **todo** firmware de controladora (fancoil, iluminação, CAG, ...) deve seguir para
ser identificado, monitorado e atualizado pelo NX-360 da mesma forma.
Implementação de referência: FC300 (`src/main.cpp`, a partir da v1.4.3).

---

## 1. Identificação (no código)

Três `#define` no topo do firmware:

```cpp
#define FIRMWARE_VERSION    "1.4.3"     // semântica MAJOR.MINOR.PATCH
#define TIPO_CONTROLADORA   "FANCOIL"   // família: FANCOIL, ILUMINACAO, CAG, ...
#define MODELO_CONTROLADORA "FC300"     // modelo do hardware/firmware
```

Regras:
- `TIPO` e `MODELO` em **MAIÚSCULAS**, só `A–Z`, `0–9` (sem espaço, acento, `|` ou `-`).
- O `MODELO` é único entre todos os tipos: dois tipos diferentes nunca usam o mesmo modelo.
- `FIRMWARE_VERSION` muda a cada binário distribuído — o OTA ignora um comando com a versão atual.

### Assinatura NXFW (dentro do .bin)

```cpp
extern "C" __attribute__((used)) const char NXFW_TAG[] =
  "NXFW|TIPO=" TIPO_CONTROLADORA "|MODELO=" MODELO_CONTROLADORA "|VERSAO=" FIRMWARE_VERSION "|";
```

- Formato exato: `NXFW|TIPO=<tipo>|MODELO=<modelo>|VERSAO=<versao>|` (texto ASCII, termina em `|`).
- O firmware **imprime** `NXFW_TAG` no log de boot — garante que o linker não descarta a string.
- Permite identificar um `.bin` sem executá-lo (script de empacotamento, NX-360 no upload).

---

## 2. Tópicos MQTT comuns

Prefixo: `TJS/<id>/`, onde `<id>` é o ID curto da placa (derivado do MAC).
"Retido" = publicado com a flag retained.

| Tópico | Direção | Retido | Conteúdo | Quando |
|---|---|---|---|---|
| `TIPO` | placa → BMS | sim | `TIPO_CONTROLADORA` (ex.: `FANCOIL`) | a cada conexão MQTT |
| `MODELO` | placa → BMS | sim | `MODELO_CONTROLADORA` (ex.: `FC300`) | a cada conexão MQTT |
| `VERSAO` | placa → BMS | sim | `FIRMWARE_VERSION` (ex.: `1.4.3`) | a cada conexão MQTT |
| `ONLINE` | placa → BMS | sim, QoS 1 | `1` conectada / `0` offline | `1` ao conectar; `0` pelo LWT (queda) ou antes de sair de propósito (modo OTA local) |
| `RESET` | placa → BMS | sim | motivo do último reinício: `POWERON`, `EXT`, `SW`, `PANIC`, `INT_WDT`, `TASK_WDT`, `WDT`, `DEEPSLEEP`, `BROWNOUT`, `SDIO`, `UNKNOWN` | a cada conexão MQTT |
| `RSSI` | placa → BMS | sim | sinal Wi-Fi em dBm (ex.: `-61`) | na conexão e a cada 5 min |
| `UPTIME` | placa → BMS | sim | minutos desde o boot | na conexão e a cada 5 min |
| `OTA/SET` | BMS → placa | **não** | `VERSAO;URL;MD5` | comando de atualização |
| `OTA/STATE` | placa → BMS | sim | andamento/resultado do OTA (abaixo) | durante o OTA e no boot após rollback |

O LWT é configurado como `TJS/<id>/ONLINE` = `0`, retido, QoS 1.

Tópicos específicos de cada tipo (ex.: `SETPOINT`, `VAG`, `PRESSAO` no fancoil) ficam fora deste
contrato e são documentados no próprio firmware. Os do FANCOIL estão na seção 6.

---

## 3. OTA remoto

### Comando — `TJS/<id>/OTA/SET`

```
VERSAO;URL;MD5
```

- `VERSAO` — versão contida no binário. Igual à atual → `ignorado: ja na versao <v>`.
- `URL` — precisa começar **exatamente** com
  `https://bmstjs.pilaresautomacao.com.br/api/firmware/files/`
  (formato do NX-360: `.../api/firmware/files/<token>/<arquivo>.bin`). O servidor precisa
  enviar `Content-Length`. Certificado validado contra ISRG Root X1 (Let's Encrypt).
- `MD5` — 32 hex do `.bin`. A imagem só é ativada se o MD5 conferir.
- Mensagem **retida** em `OTA/SET` é ignorada (evita reflash a cada reconexão).

### Nome do arquivo e trava de modelo

- Nome do `.bin`: `<modelo em minúsculas>-<versao>.bin` — ex.: `fc300-1.4.3.bin`.
- O firmware recusa a URL se o último segmento (nome do arquivo, sem `?query`) não começar com
  `<modelo minúsculo>-`, publicando `erro: firmware de outro modelo (<arquivo>)`.
- Isso impede gravar o firmware de iluminação numa placa de fancoil, etc.

### Estados — `TJS/<id>/OTA/STATE` (retido)

| Payload | Significado |
|---|---|
| `baixando <versao>` | download iniciado |
| `ok <versao> reiniciando` | imagem gravada e MD5 conferido; reinicia na nova versão |
| `erro: formato VERSAO;URL;MD5` | payload malformado |
| `erro: url nao permitida` | URL fora do prefixo permitido |
| `erro: firmware de outro modelo (<arquivo>)` | trava de modelo |
| `erro: md5 invalido` | MD5 não tem 32 hex |
| `erro: <detalhe>` | falha no download/gravação (`http 404`, `sem Content-Length`, `imagem recusada: ...`, ...) |
| `ignorado: ja na versao <v>` | já está nessa versão |
| `ignorado: validacao pos-OTA em andamento` | a versão atual ainda não foi confirmada |
| `rollback` | a imagem nova não conectou ao MQTT e a placa voltou para a versão anterior |

### Validação pós-OTA (rollback)

Após gravar, a nova imagem fica "pendente". No boot ela é confirmada 60 s depois da primeira
conexão MQTT. Se não conectar ao MQTT em 5 min (contados a partir da conexão Wi-Fi), a placa
volta para a partição anterior e publica `rollback`. Falta de Wi-Fi não provoca rollback.

---

## 4. Empacotamento (`scripts/publicar_ota.py`)

Roda automaticamente após cada `pio run` (`extra_scripts = post:scripts/publicar_ota.py`):

1. Lê `TIPO_CONTROLADORA`, `MODELO_CONTROLADORA` e `FIRMWARE_VERSION` do `src/main.cpp`.
2. Confere se a assinatura `NXFW|TIPO=...|MODELO=...|VERSAO=...|` está dentro do `.bin`
   (falha o build se não estiver).
3. Copia para `dist/<modelo minúsculo>-<versao>.bin`.
4. Imprime MD5 (usado no comando) e SHA-256, e a linha pronta para `OTA/SET`.

---

## 5. Checklist para um novo firmware

- [ ] `FIRMWARE_VERSION`, `TIPO_CONTROLADORA`, `MODELO_CONTROLADORA` definidos
- [ ] `NXFW_TAG` declarado e impresso no boot
- [ ] Publica `TIPO`, `MODELO`, `VERSAO`, `RESET`, `RSSI`, `UPTIME`, `ONLINE` (retidos) na conexão
- [ ] LWT em `ONLINE` = `0`
- [ ] `RSSI` e `UPTIME` a cada 5 min
- [ ] `OTA/SET` com `VERSAO;URL;MD5`, prefixo de URL, trava de modelo, MD5 e rollback
- [ ] Estados de `OTA/STATE` com os mesmos textos da tabela acima
- [ ] `scripts/publicar_ota.py` copiado e gerando `<modelo>-<versao>.bin`
- [ ] Comandos JSON (`.../SET`) seguem o padrão da seção 6: mensagem retida ignorada, erro
      devolvido no tópico de configuração com `"erro":"<motivo curto>"`, NVS gravada só se mudar

---

## 6. Tópicos específicos do tipo FANCOIL (FC300, a partir da v1.4.4)

Não fazem parte do contrato comum, mas servem de modelo para comandos JSON de outros tipos
(ex.: perfis de dimerização na iluminação).

| Tópico | Direção | Retido | Conteúdo |
|---|---|---|---|
| `PID/SET` | BMS → placa | **não** (retido é ignorado) | perfil de sintonia em JSON (abaixo) |
| `PID/CONFIG` | placa → BMS | sim | sintonia em uso — na conexão, após cada `PID/SET` |
| `PID/STATE` | placa → BMS | sim | telemetria do controle a cada ciclo do PID (30 s) |
| `SETPOINT/STATE` | placa → BMS | sim | setpoint efetivo, 1 casa (`23.0`) — na conexão e sempre que muda (`SETPOINT/SET`, agenda) |

### Saídas analógicas (KC868-A4)

| Saída | GPIO | Uso | Faixa |
|---|---|---|---|
| AO1 | GPIO26 | atuador da VAG (`VAG`, `vag` no `PID/STATE`) | 0–10 V = 0–100 % |
| AO2 | GPIO25 | inversor do ventilador (`PRESSAO`) | 0–10 V = 0–100 % |

Mapa corrigido na v1.4.7 (até a v1.4.6 a VAG estava no GPIO25).

### `PID/SET`

```json
{"perfil":"NORMAL"}
{"perfil":"PERSONALIZADO","kp":45,"ti":250,"taxa":12,"zm":0.2}
```

| Perfil | Kp (%/°C) | Ti (s) | Taxa (%/ciclo) | Zona morta (°C) |
|---|---|---|---|---|
| `MUITO_LENTO` | 45 | 290 | 10 | 0.20 |
| `LENTO` | 52 | 190 | 14 | 0.20 |
| `NORMAL` (padrão de fábrica) | 60 | 120 | 20 | 0.15 |
| `RAPIDO` | 75 | 80 | 30 | 0.15 |
| `MUITO_RAPIDO` | 90 | 50 | 40 | 0.10 |
| `PERSONALIZADO` | 5–150 | 30–3600 | 1–50 | 0.10–1.00 |

- Tabela da v1.4.6 (perfis mais rápidos que a da v1.4.4/1.4.5).
- Perfil pronto: `kp/ti/taxa/zm` do payload são ignorados (vale a tabela). Ao carregar da NVS,
  perfil pronto também usa os valores da tabela do firmware em execução — um update que muda a
  tabela vale para todas as placas com perfil pronto; só o `PERSONALIZADO` guarda valores próprios.
- Perfil desconhecido, campo faltando, JSON inválido ou valor fora da faixa: nada é aplicado e
  `PID/CONFIG` é republicado com a configuração atual mais `"erro"`. Motivos:
  `json invalido`, `perfil ausente`, `perfil desconhecido`, `campo ausente: <campo>`,
  `<campo> invalido`, `<campo> fora da faixa (<min>-<max>)`.
- Troca sem salto: a saída da VAG continua de onde estava.
- Persistido no NVS (namespace `pid`) **só ao receber `PID/SET`**; sem dado salvo ou inválido →
  padrão de fábrica: `NORMAL` (desde a v1.4.5; antes `MUITO_LENTO`). O padrão nunca é gravado
  no boot, então quem nunca recebeu comando segue o padrão do firmware em execução.
- Um `PID/SET` igual à sintonia atual não regrava o NVS — exceto se a placa ainda estiver no
  padrão sem nada salvo: aí grava uma vez, para a escolha explícita sobreviver a futuras
  mudanças de padrão.

### `PID/CONFIG`

```json
{"perfil":"NORMAL","kp":40,"ti":300,"taxa":10,"zm":0.2,"ciclo":30}
{"perfil":"NORMAL","kp":40,"ti":300,"taxa":10,"zm":0.2,"ciclo":30,"erro":"perfil desconhecido"}
```

`ciclo` = período do PID em segundos (fixo).

### `PID/STATE`

```json
{"sp":18.0,"temp":23.4,"bruta":23.5,"erro":-5.4,"pid":42.0,"vag":35.0,"zm":false,"rele":true}
```

| Campo | Significado |
|---|---|
| `sp` | setpoint (°C) |
| `temp` | temperatura filtrada (°C); `null` com o sensor em falha |
| `bruta` | leitura bruta do DS18B20 (°C); em falha mostra o código do sensor (`-127.0` / `85.0`) |
| `erro` | `sp − temp` (°C); `null` com o sensor em falha |
| `pid` | saída calculada pelo PID (%); `0` com o relé desligado |
| `vag` | saída aplicada na AO1 (%); `0` com o relé desligado |
| `zm` | `true` se está dentro da zona morta (saída mantida) |
| `rele` | fancoil ligado |
