# CT_HVAC_WIFI — Controladora de fancoil FC300

Firmware ESP32 (placa KC868-A4) da controladora de fancoil **FC300** (tipo `FANCOIL`), integrada
ao supervisório NX-360 por MQTT. Código em `src/main.cpp`.

- Controle de temperatura por PID na VAG (AO1, 0–10 V), com perfis de sintonia ajustáveis remotamente
- Pressão/inversor do ventilador (AO2, 0–10 V), relés do fancoil e exaustor
- Agenda semanal vinda do supervisório, estado AUTOMÁTICO/FORÇADO
- OTA remoto por MQTT (com trava de modelo e rollback) e OTA local pelo botão
- Recuperação de rede em degraus e watchdog

Padrão comum de identificação/OTA de todas as controladoras: [docs/PADRAO_CONTROLADORAS.md](docs/PADRAO_CONTROLADORAS.md).

## Saídas analógicas (KC868-A4)

| Saída | GPIO | Uso em campo | Faixa |
|---|---|---|---|
| AO1 | GPIO26 | atuador da VAG | 0–10 V = 0–100 % |
| AO2 | GPIO25 | inversor do ventilador (pressão) | 0–10 V = 0–100 % |

Até a v1.4.6 os pinos estavam invertidos no firmware (VAG no GPIO25); corrigido na v1.4.7.

## Tópicos MQTT

Prefixo `TJS/<id>/` (`<id>` = 4 últimos caracteres hex do MAC). Tópicos de estado são retidos.

| Tópico | Direção | Conteúdo |
|---|---|---|
| `TEMPERATURA` | placa → BMS | temperatura filtrada (°C) ou `ERRO` |
| `VAG` | placa → BMS | abertura da VAG aplicada (%) |
| `MODO`, `STATUS` | placa → BMS | entradas digitais |
| `CMD/SET` → `CMD` | comando → estado | liga/desliga o fancoil (`true`/`false`) |
| `ESTADO/SET` → `ESTADO` | comando → estado | AUTOMÁTICO (`true`) / FORÇADO (`false`) |
| `SETPOINT/SET` | BMS → placa | setpoint (10–35 °C) |
| `SETPOINT/STATE` | placa → BMS | setpoint efetivo, 1 casa — na conexão e sempre que muda |
| `PRESSAO/SET` → `PRESSAO` | comando → estado | pressão do inversor (%) |
| `SCHEDULE/SET` → `SCHEDULE/STATE` | comando → estado | agenda semanal |
| `PID/SET` | BMS → placa | perfil de sintonia em JSON (retido é ignorado) |
| `PID/CONFIG` | placa → BMS | sintonia em uso (JSON) |
| `PID/STATE` | placa → BMS | telemetria do controle a cada 30 s (JSON) |
| `TIPO`, `MODELO`, `VERSAO` | placa → BMS | identificação |
| `ONLINE`, `RESET`, `RSSI`, `UPTIME` | placa → BMS | presença e diagnóstico |
| `OTA/SET` → `OTA/STATE` | comando → estado | OTA remoto (`VERSAO;URL;MD5`) |

## Perfis de PID

Escolhidos em `TJS/<id>/PID/SET` e gravados no NVS. A troca é sem salto na VAG.

**Padrão de fábrica: NORMAL** (desde a v1.4.5; até a v1.4.4 era `MUITO_LENTO`). O padrão não é
gravado no NVS: a placa que nunca recebeu `PID/SET` sempre segue o padrão do firmware que está
rodando; a que já recebeu mantém o perfil salvo, mesmo após atualizar o firmware.

| Perfil | Kp (%/°C) | Ti (s) | Taxa (%/ciclo) | Zona morta (°C) |
|---|---|---|---|---|
| `MUITO_LENTO` | 45 | 290 | 10 | 0.20 |
| `LENTO` | 52 | 190 | 14 | 0.20 |
| `NORMAL` (padrão de fábrica) | 60 | 120 | 20 | 0.15 |
| `RAPIDO` | 75 | 80 | 30 | 0.15 |
| `MUITO_RAPIDO` | 90 | 50 | 40 | 0.10 |
| `PERSONALIZADO` | 5–150 | 30–3600 | 1–50 | 0.10–1.00 |

Tabela da v1.4.6 (perfis mais rápidos). Para perfil pronto, a NVS guarda só o nome e os valores
vêm sempre da tabela do firmware: placas com perfil pronto salvo pegam os valores novos no update.
Ciclo do PID fixo em 30 s, derivativo desligado.

```sh
mosquitto_pub -h <broker> -t 'TJS/<id>/PID/SET' -m '{"perfil":"NORMAL"}'
mosquitto_pub -h <broker> -t 'TJS/<id>/PID/SET' -m '{"perfil":"PERSONALIZADO","kp":45,"ti":250,"taxa":12,"zm":0.2}'
mosquitto_sub -h <broker> -t 'TJS/<id>/PID/#' -v
```

Formatos de `PID/CONFIG`, `PID/STATE` e mensagens de erro: seção 6 de
[docs/PADRAO_CONTROLADORAS.md](docs/PADRAO_CONTROLADORAS.md).

## Build e OTA

`pio run` compila e o `scripts/publicar_ota.py` gera `dist/fc300-<versao>.bin`, confere a
assinatura NXFW e imprime o MD5 e a linha pronta para `OTA/SET`.
