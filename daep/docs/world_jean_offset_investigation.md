# Investigacao do Offset no `world_jean`

## Resumo

Durante a investigacao do `world_jean`, o problema de offset nao estava no `worlds/world_jean.world` em si, nem no `world_jean_tree.obj` ja extraido para o modelo final. A causa raiz estava no OBJ-fonte usado no pipeline de geracao:

- arquivo-fonte: `ground_truth/world_jean/3d_scene_prepared_10x10.obj`
- grupo de referencia usado pelo gerador: `Wallnut_.002_Mesh`

Esse grupo ja estava deslocado no proprio OBJ-fonte, e o script de geracao assumia incorretamente que essa arvore de referencia ja estava centrada em `(0,0,0)`.

## Sintoma observado

O bug aparecia assim:

- a arvore de referencia era tratada como se estivesse em `pose = 0 0 0 0 0 0`
- no entanto, a geometria dela no OBJ-fonte nao estava centrada em `(0,0)`
- resultado: a primeira arvore parecia "andar" em relacao ao spawn esperado do drone, apesar da pose no `.world` estar em zero

## Evidencia numerica do OBJ bugado

O grupo `Wallnut_.002_Mesh` foi medido diretamente em `ground_truth/world_jean/3d_scene_prepared_10x10.obj`.

Medidas globais do grupo:

- `x_avg ~= 1.2770746921468`
- `y_avg ~= 1.11115701074348`
- `z_min ~= 0.005359`

Medidas da base da arvore, usando os vertices mais baixos (`z <= z_min + 0.2`):

- `low_x_avg ~= 0.91685015625`
- `low_y_avg ~= 1.05630365625`
- `low_x_min ~= 0.767033`
- `low_x_max ~= 1.066667`
- `low_y_min ~= 0.906487`
- `low_y_max ~= 1.20612`

Conclusao: o pe da arvore de referencia no OBJ-fonte nao estava em `(0,0)`. O offset lateral embutido era de aproximadamente:

- `dx ~= +0.91685 m`
- `dy ~= +1.05630 m`

## Comparacao com o mesh final extraido

Depois da extracao, o mesh final `models/world_jean_tree/meshes/world_jean_tree.obj` ficou centrado corretamente.

Na base da arvore extraida:

- `low_x_avg ~= 0`
- `low_y_avg ~= 0`
- `z_min ~= 0.005359`

Isso mostra que o problema nao era "a arvore final do modelo Gazebo esta ruim". O problema estava antes, na relacao entre:

- arvore de referencia dentro do OBJ-fonte
- calculo das poses no script

## Script envolvido

O script investigado foi o gerador que:

1. le `3d_scene_prepared_10x10.obj`
2. usa `Wallnut_.002_Mesh` como referencia geometrica
3. calcula as poses de todas as arvores por transformacao rigida 2D
4. escreve o `.world`

Ponto critico da logica:

- `compute_tree_poses()` calcula as poses relativas usando `Wallnut_.002_Mesh`
- para a arvore de referencia, isso naturalmente produz `tx=0`, `ty=0`, `yaw=0`
- o script assume que "pose zero" significa "arvore centrada na origem"
- essa suposicao era falsa no OBJ-fonte

Em outras palavras: o script nao criou offset do nada, mas propagou um offset que ja existia no OBJ-fonte.

## Causa raiz

A causa raiz foi:

- a arvore de referencia `Wallnut_.002_Mesh` no `3d_scene_prepared_10x10.obj` nao estava normalizada para a origem local
- o script de geracao assumia que ela estava

Formula curta:

- OBJ-fonte com referencia deslocada
- mais script que nao recentra a referencia
- igual a `.world` com offset aparente

## Como a investigacao foi fechada

A investigacao mostrou tres fatos importantes:

1. O `.pcd` era coerente com a versao original do trabalho, inclusive com uma arvore em `0,0`.
2. O `worlds/world_jean.world` original tambem mantinha a `world_jean_tree_002` em `0,0`.
3. O offset antigo nao era prova de que "a arvore em 0,0 estava errada"; a prova foi que o OBJ-fonte da referencia estava deslocado.

## Correcao operacional feita no repositorio

Para manter fidelidade com os arquivos originais do trabalho:

- a arvore `world_jean_tree_002` foi mantida/restaurada em `0,0` no `worlds/world_jean.world`
- o `ground_truth/world_jean/world_jean_ground_truth.csv` foi sincronizado de volta para 20 arvores
- o `ground_truth/world_jean/world_jean_ground_truth.svg` tambem foi sincronizado para mostrar novamente a arvore na origem

Essa sincronizacao corrige a consistencia entre:

- mundo
- GT
- nuvem `.pcd`

## Correcao definitiva recomendada

Para eliminar esse tipo de bug de offset no pipeline, ha duas opcoes corretas:

### Opcao A: corrigir no Blender / OBJ-fonte

Recentrar a arvore de referencia para que o pe do tronco fique em `(0,0,0)` antes da exportacao.

Vantagem:

- o asset nasce correto
- o script pode continuar simples

### Opcao B: blindar o script

Modificar o gerador para:

- calcular o pivot/base da arvore de referencia
- subtrair esse offset da geometria exportada
- ou compensar esse offset explicitamente ao escrever as poses

Vantagem:

- o pipeline fica robusto mesmo se o OBJ-fonte vier deslocado

## Conclusao final

O offset bugado vinha do `3d_scene_prepared_10x10.obj`, mais especificamente do grupo `Wallnut_.002_Mesh`, cujo pe estava aproximadamente em:

- `x ~= 0.91685`
- `y ~= 1.05630`

O script de geracao assumia que esse grupo estava centrado na origem e, por isso, propagava o erro para o `.world`.

Resumo em uma linha:

> O bug de offset nao era da pose do `world`, e sim da referencia geometrica no OBJ-fonte combinada com uma suposicao errada no script.
