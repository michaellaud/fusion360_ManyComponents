# TK Mesh Benchmark — Transfer Kinematic: CAD × malhas no Fusion

Script de diagnóstico **independente** (não altera o SmartToolsPro nem o fluxo de produção) que implementa o roteiro de `Transfer_Kinematic_teste_malhas_Fusion.md`:

| Modo | O que faz por quadro |
|---|---|
| **A — CAD atual** | `occurrence.transform2 = m` para cada ocorrência móvel + `viewport.refresh()` + `adsk.doEvents()` |
| **B — CAD em lote** | uma única chamada `rootComponent.transformOccurrences(occs, mats, ignoreJoints)` |
| **C — Custom Graphics** | ocorrências móveis ocultas; malhas criadas **uma vez**, e por quadro só muda `mesh.transform` |

A trajetória **não é recalculada**: o script lê o JSON já exportado pelo Transfer Kinematic e escolhe o quadro pelo tempo da simulação (se o Fusion atrasar, quadros visuais são pulados sem desacelerar a trajetória).

## Instalação

1. Copie a pasta `TK_MeshBenchmark` para
   `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\TK_MeshBenchmark`
   (é um **Script**, não um Add-In; não precisa ficar dentro do SmartToolsPro).
2. No Fusion: **Utilities › Add-Ins › Scripts and Add-Ins** → aba *Scripts* → `TK_MeshBenchmark` → *Run*.

## Uso em duas etapas

### Etapa 1 — inspeção (padrão: `"inspect_only": true`)

Abra uma **cópia** da montagem, ajuste `motion_json` no `config.json` e rode. Em `results/<data_hora>/` o script grava:

- `log.txt`: formato detectado do JSON, número de trilhas/amostras, duração, quantos IDs casaram com ocorrências e a **diferença entre a pose inicial do JSON e a pose CAD atual**. Se essa diferença for grande, a unidade, o eixo ou o referencial está errado.
- `json_schema.txt`: estrutura resumida do JSON.
- `occurrences.txt`: `fullPathName` e componente de todas as ocorrências (para montar o `id_map`).

Ajuste até os IDs casarem e a diferença de pose ficar ~0:

| Chave | Uso |
|---|---|
| `unit_scale_to_cm` | 0.1 se o JSON estiver em mm, 100 se em metros, 1 se em cm |
| `axis_conversion` | `yup_to_zup` se o JSON foi escrito para three.js/GLB (Y para cima) |
| `matrix_order` | `column` para matrizes do three.js/glTF (`matrix.elements`) |
| `quaternion_order` | `xyzw` (three.js/glTF) ou `wxyz` |
| `transform_mode` | `absolute` (matriz no mundo), `relative_to_initial` (delta × pose inicial) ou `incremental` |
| `id_map` | `{"id_do_json": "Montagem:1+Garra:2"}` para IDs que não casam automaticamente ou são ambíguos (peças repetidas) |
| `adapter_module` | nome de um `.py` nesta pasta com `load(path, cfg) -> tk_core.Motion`, caso o JSON tenha formato que a detecção automática não entenda |

Formatos reconhecidos automaticamente:
- `{"frames":[{"time":..,"transforms"|"objects"|"parts":{id: T} ou [{"id":..,...T}]}]}`
- `{"tracks"|"objects"|"parts":{id:{"times":[..],"matrices":[T..]}}}` ou `positions`/`rotations`

`T` pode ser: matriz 16 ou 4×4, `{matrix}`, `{position, quaternion|rotation|euler}`, `[x,y,z,qx,qy,qz,qw]`.

### Etapa 2 — medição

`"inspect_only": false`. Comece com `max_moving` pequeno (5–10 conjuntos representativos) e depois aumente até a montagem real inteira (`0` = todos).

Cada modo roda com o **mesmo JSON, câmera, viewport, duração e FPS alvo**: `warmup_s` de aquecimento (descartado) + `duration_s` medidos (use 30–60 s). Saídas:

- `summary.md`: tabela A/B/C (FPS, intervalo médio/p95/máx entre quadros, tempo de aplicar, `refresh()`, `doEvents`, quadros perdidos, preparo, reset, memória) + configuração do PC + estatísticas das malhas (triangulação, criação, triângulos, geometrias únicas, Δ memória).
- `frames.csv`: tempo de cada etapa por quadro.
- `results.json`: tudo, incluindo IDs sem par/ambíguos.
- `<modo>_t<tempo>.png`: capturas em `snapshot_fractions` do percurso, para comparar com o viewer GLB.

A medida principal é o **intervalo entre inícios de quadro** (`Quadro médio`/`p95`): ele inclui a renderização que acontece fora de `refresh()` (dentro de `doEvents`). O tempo de `refresh()` sozinho **não** é o custo de renderização.

Segurança: confirmação antes de começar; `require_doc_name_contains` (ex.: `"copia"`) impede rodar no arquivo original; posições, visibilidade, câmera e snapshot pendente são restaurados e os gráficos apagados em `finally`, inclusive em erro ou ao **Cancelar** na barra de progresso.

### Variações quando C ainda estiver lento

- `mesh_quality`: `low` | `normal` | `high` (número de triângulos)
- `hide_fixed: true`: oculta também os componentes fixos (cena só com malhas)
- `update_every_n_frames`: frequência de atualização
- `max_moving`: número de entidades gráficas
- `pace_to_target: false`: mede o FPS máximo sem limitar ao alvo

## Detalhes de implementação do modo C

- Triangulação por `body.meshManager.createMeshCalculator()` nos corpos **nativos** do componente: as coordenadas ficam no referencial do componente, então `mesh.transform` = matriz mundial da ocorrência vinda do JSON.
- **Cache por geometria**: várias ocorrências do mesmo componente triangulam uma vez só; cada instância tem suas próprias malhas e matriz (identidade preservada).
- Subcomponentes são incorporados à malha do conjunto móvel, exceto os que também têm trilha própria no JSON (esses são animados separadamente, sem duplicar).
- Corpos agrupados por cor da aparência → poucas entidades por instância. Seleção desativada.
- Nenhum GLB é carregado: Custom Graphics não lê GLB diretamente.

## Pontos para validar no Fusion real

1. Se o Transfer Kinematic grava as poses em relação ao **pai** (e não à raiz), use `adapter_module` para compor com as matrizes dos pais.
2. `transform2` em ocorrências proxy (obtidas de `rootComponent.allOccurrences`) é interpretado no contexto da raiz; confirme com a diferença de pose do log.
3. Modo B: com `ignore_joints: true` as juntas são ignoradas durante o movimento (a pose pode violar as juntas, e é isso que queremos numa reprodução). Com `false` o Fusion resolve as juntas e o resultado pode divergir do JSON e ficar mais lento. `transformOccurrences` exige uma versão recente do Fusion; se não existir, o modo B registra erro e os outros continuam.
4. O modo A reproduz o padrão comum (`transform2` por ocorrência + refresh). Se o Transfer Kinematic faz outra sequência (ex.: `design.computeAll`, captura de posição, `timeline`), copie essa sequência para `CadIndividual.apply` para que A represente o comportamento atual.

## Critério de decisão (do documento)

Reprodução estável a 24–30 FPS (p95 do intervalo ≤ ~42 ms) na montagem real, sem travamento perceptível ao iniciar (`Preparo s`), pausar/encerrar (`Reset s`), e com preparo e memória das malhas aceitáveis.

## Testes (fora do Fusion)

```
python -m unittest tests/test_tk_core.py
```
Cobrem leitura dos formatos de JSON, unidades/eixos, matrizes, correspondência de IDs e estatísticas. A parte que usa `adsk` só pode ser validada dentro do Fusion.
