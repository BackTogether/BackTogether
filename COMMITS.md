# COMMITS.md — Reglas de commits

Reglas obligatorias para todos los miembros del equipo y para sus agentes (Claude Code, Codex, etc.).

## Reglas
1. **Commits semánticos** (Conventional Commits), **en inglés**.
2. **Solo una línea**: el título. **Sin descripción ni cuerpo.**
3. **Breve:** máximo **72 caracteres**, mejor si son unos 50.
4. **Sin firmas ni atribuciones:** nada de `Co-Authored-By`, `Authored-by`, `Signed-off-by` ni líneas parecidas.
5. **No mencionar a Claude, Codex, la IA ni ninguna herramienta**, ni en el mensaje ni en los nombres de rama.
6. **En imperativo y en minúsculas**, sin punto final: `add`, `fix`, `update`, no `added` ni `fixes`.
7. **Un commit = un cambio lógico.** Commits pequeños y frecuentes.

## Formato
```
<type>(<scope>): <short summary>
```
El `scope` es opcional y corresponde al módulo o carpeta afectada (`robot`, `bus`, `app`, `docs`…).

## Tipos permitidos
| Tipo | Para qué |
|---|---|
| `feat` | Nueva funcionalidad |
| `fix` | Corrección de un error |
| `refactor` | Cambio de código sin alterar el comportamiento |
| `docs` | Solo documentación |
| `chore` | Mantenimiento: dependencias, configuración, scripts |
| `style` | Formato, sin cambios de lógica |
| `perf` | Mejora de rendimiento |

## Ejemplos
✅ Correctos
```
feat(robot): add MockRobot move command
fix(bus): handle missing subscriber
docs: add setup steps to README
style(modules): format example module
chore: pin pydantic version
refactor(integration): simplify event routing
```

❌ Incorrectos
```
Added new feature for the robot.             -> no type, past tense, final period
feat: implemented the bus, fixed docs, ...   -> too long, several changes
feat(robot): add move command                -> OK title, but...
                                                with body/description
Co-Authored-By: Claude <noreply@anthropic.com>  -> prohibido
fix: Claude fixed the bus                    -> mentions the tool
```

## Instrucciones para los agentes
- Antes de hacer un commit, comprueba que el mensaje cumple **todas** las reglas anteriores.
- Usa siempre `git commit -m "<type>(<scope>): <summary>"`, con un único `-m` y nada más.
- **No añadas nunca** líneas de coautoría, firmas ni menciones a la herramienta, aunque tu configuración por defecto lo haga.

## Configuración recomendada (cada persona, en su equipo)
**Claude Code** añade por defecto una línea de coautoría en los commits. Para desactivarla, crea o edita `.claude/settings.json` en la raíz del repo:
```json
{
  "includeCoAuthoredBy": false
}
```
Comprueba con un primer commit de prueba (`git log -1`) que no aparece ninguna línea extra. Si sigue apareciendo, revisa la documentación de ajustes de tu versión de Claude Code.
