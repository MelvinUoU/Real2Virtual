# Real2Virtual

## Guia de Conversão de Dados Sleep AI (H5) para MuJoCo CSV

Este repositório contém um script que converte arquivos `.h5` gerados pelo Sleep AI (rastreamento de nós 2D) em arquivos `.csv` compatíveis com MuJoCo (cada nó com X, Y e Z = 0.0).

- `convert_h5_to_csv.py`: script Python que faz a conversão
- `H5_test.h5` e `labels_Drosoph1.h5`: exemplos de entrada
- `H5_test_mujoco.csv` e `labels_Drosoph1_mujoco.csv`: resultados da conversão

---

## 1. Pré-requisitos

Antes de executar o script, instale as bibliotecas necessárias:

```bash
pip install h5py pandas
```

## 2. Estrutura dos Arquivos

Coloque no mesmo diretório:
- `convert_h5_to_csv.py`
- `H5_test.h5`
- `labels_Drosoph1.h5`

Ao rodar o script, serão gerados:
- `H5_test_mujoco.csv`
- `labels_Drosoph1_mujoco.csv`

Cada CSV terá colunas neste estilo:

```
frame,
hindlegR1_x, hindlegR1_y, hindlegR1_z,
thorax1_x, thorax1_y, thorax1_z,
head1_x, head1_y, head1_z,
... (para cada node_name)
```

## 3. Como Executar

Clone este repositório:

```bash
git clone https://github.com/SEU_USUARIO/SEU_REPOSITORIO.git
cd SEU_REPOSITORIO
```

Verifique se os arquivos `.h5` estão neste diretório. Se não, faça upload ou mova-os para cá.

Execute o script:

```bash
python convert_h5_to_csv.py
```

Após a execução, você verá no terminal:

```
Arquivo salvo em: H5_test_mujoco.csv
Arquivo salvo em: labels_Drosoph1_mujoco.csv
```

Os arquivos CSV estarão prontos para uso com MuJoCo.

## 4. Como os Dados São Processados

- Leitura do H5: Usamos `h5py.File(h5_path, 'r')`.
    - `node_names = f['node_names'][:]` (lista de strings com cada nó detectado).
    - `tracks = f['tracks'][0]` resulta num array de shape (2, num_nodes, num_frames), onde índice 0 = X e 1 = Y.
    - `occupancy = f['track_occupancy'][:,0]` indica, para cada frame, se há dados válidos (1 ou 0).
- Filtragem de Frames Válidos: Percorremos frame de 0 a num_frames-1. Se `occupancy[frame] == 1`, coletamos os pares (X, Y) de cada nó. Caso x ou y seja NaN, substituímos por 0.0. Adicionamos [x, y, z=0.0] para cada nó àquela linha.
- Montagem do DataFrame: Definimos colunas: `["frame"] + [f"{node}_{axis}" for node in node_names for axis in ('x', 'y', 'z')]`. Cada linha do DataFrame associa o número do frame a todos os (x, y, z) dos nós. No final, chamamos `df.to_csv(output_csv_path, index=False)` para salvar.

Formato Final do CSV:

Primeiro cabeçalho:
```
frame,hindlegR1_x,hindlegR1_y,hindlegR1_z,thorax1_x,thorax1_y,thorax1_z,head1_x,head1_y,head1_z,...
```
Cada linha:
```
103,698.673564,743.757148,0.0,0.0,0.0,0.0,641.424267,771.077756,0.0,...
```
(só aparecem linhas cujo occupancy == 1).

## 5. Exemplo de Integração com MuJoCo

### Opção A – Script Dinâmico em Python

Caso você queira “alimentar” os marcadores (mocap_pos) em tempo de simulação:

```python
import mujoco_py     # ou usar "import mujoco"
import numpy as np
import pandas as pd

# 1) Carrega modelo e sim
model = mujoco_py.load_model_from_path('seu_modelo.xml')
sim   = mujoco_py.MjSim(model)

# 2) Lê CSV num DataFrame
df = pd.read_csv('H5_test_mujoco.csv')

# 3) Extrai nomes dos nós (em mesma ordem dos <marker> no XML)
node_names = [col[:-2] for col in df.columns if col.endswith('_x')]

# 4) Loop de frames
for _, row in df.iterrows():
    for i, node in enumerate(node_names):
        x = float(row[f'{node}_x'])
        y = float(row[f'{node}_y'])
        z = float(row[f'{node}_z'])
        sim.data.mocap_pos[i] = np.array([x, y, z])  # i = índice do marcador
    sim.step()
    # opcional: sim.render() ou gravar imagem
```
Atenção: a ordem de node_names deve coincidir exatamente com a ordem dos marcadores declarados no seu MJCF.

Se quiser converter pixels para metros, multiplique x e y por um fator de escala (ex.: scale = 0.001).

### Opção B – Gerar Arquivo “.mocap” Texto

MuJoCo aceita um arquivo texto (“mocapfile”) neste formato:

```
<num_markers>
<marker1> <marker2> ... <markerN>
<frame_1_coords>: x1 y1 z1  x2 y2 z2  ... xN yN zN
<frame_2_coords>
...
```

Para converter o CSV num .mocap:

```python
import pandas as pd

df = pd.read_csv('H5_test_mujoco.csv')
node_names = [col[:-2] for col in df.columns if col.endswith('_x')]

with open('H5_test_mujoco.mocap', 'w') as f:
    # Escreve número de marcadores (= len(node_names))
    f.write(f"{len(node_names)}\n")
    # Escreve nomes separados por espaço
    f.write(" ".join(node_names) + "\n")
    # Escreve cada linha de frame
    for _, row in df.iterrows():
        coords = []
        for node in node_names:
            coords.extend([
                row[f"{node}_x"], 
                row[f"{node}_y"], 
                row[f"{node}_z"]
            ])
        # Transforma em string única e grava
        linha = " ".join(str(val) for val in coords)
        f.write(linha + "\n")
```

Depois, no seu XML MJCF:

```xml
<mujoco>
  ...
  <asset>
    <!-- Declare cada marcador pelo nome -->
    <marker name="hindlegR1" size="0.01" rgba="1 0 0 1"/>
    <marker name="thorax1"  size="0.01" rgba="0 1 0 1"/>
    <marker name="head1"    size="0.01" rgba="0 0 1 1"/>
    <!-- demais marcadores -->
  </asset>

  <worldbody>
    <body name="mocap_body" mocap="true">
      <marker name="hindlegR1" pos="0 0 0"/>
      <marker name="thorax1"  pos="0 0 0"/>
      <marker name="head1"    pos="0 0 0"/>
      <!-- demais marcadores -->
    </body>
  </worldbody>

  <mocap>
    <!-- Aponta para o arquivo de texto gerado -->
    <mocapfile file="H5_test_mujoco.mocap"/>
  </mocap>
</mujoco>
```

## 6. Explicação Resumida de Como Funciona

- Arquivos H5 do Sleep AI: Cada H5 contém:
    - `node_names`: vetor de bytes com nomes de cada articulação detectada.
    - `tracks`: array (1, 2, num_nodes, num_frames) com coordenadas (X, Y).
    - `track_occupancy`: vetor indicando se “há dado válido” em cada frame.
- Conversão para CSV: Percorremos todos os frames. Se `occupancy[frame] == 1`, lemos (X, Y) de cada nó; criamos colunas node_x, node_y, node_z=0.0. Salvamos frame + todas as coordenadas (z fixo como 0) num CSV.
- Uso em MuJoCo: MuJoCo exige, para mocap, ou que o usuário atualize sim.data.mocap_pos diretamente (via Python) ou que forneça um arquivo mocapfile texto. Tanto no modo dinâmico (script Python) quanto no modo estático (.mocap no XML), cada marcador deve estar declarado no MJCF na mesma ordem em que os dados aparecem no CSV/arquivo de texto.

**Por que Z = 0.0?**

O Sleep AI faz detecção apenas em imagens 2D, portanto não há informação de profundidade (Z). Definimos Z = 0 para que o MuJoCo posicione os marcadores em um plano. Se você quiser variar Z, é necessário extender o sistema para estimar profundidade (ex.: sistema de múltiplas câmeras).