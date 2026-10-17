#!/usr/bin/env bash
# SessionStart: traz do GitHub o que foi feito em outro PC.
# Só fast-forward (--ff-only): nunca mescla nem sobrescreve alterações locais —
# se não der para aplicar limpo, apenas avisa.
cd "${CLAUDE_PROJECT_DIR:-.}" || exit 0
export GIT_TERMINAL_PROMPT=0          # sem login pendente: falha em vez de travar

antes=$(git rev-parse HEAD 2>/dev/null) || exit 0
if out=$(timeout 30 git pull --ff-only --tags 2>&1); then
  depois=$(git rev-parse HEAD)
  if [ "$antes" = "$depois" ]; then
    msg="GitHub: já atualizado ($(git log -1 --format='%h %s'))"
  else
    msg="GitHub: baixados $(git rev-list --count "$antes..$depois") commit(s) de outro PC → agora em $(git log -1 --format='%h %s')"
  fi
else
  msg="GitHub: git pull NÃO aplicado — $(printf '%s' "$out" | tail -n 1)"
fi

msg=$(printf '%s' "$msg" | tr -d '"\\' | tr '\n\r\t' '   ')
printf '{"systemMessage":"%s","hookSpecificOutput":{"hookEventName":"SessionStart","additionalContext":"%s"}}\n' "$msg" "$msg"
