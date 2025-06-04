import h5py
import numpy as np
import pandas as pd

def convert_h5_to_csv(h5_path, output_csv_path):
    """
    Lê um arquivo H5 gerado pelo Sleep AI (contendo rastreamento 2D de nós)
    e salva num CSV no formato (frame, node1_x, node1_y, node1_z, node2_x, node2_y, node2_z, ...).
    O eixo Z é definido como 0.0, pois o Sleep AI entrega apenas X e Y.
    """
    with h5py.File(h5_path, 'r') as f:
        # 1) Extrair nomes dos nós (node_names)
        node_names = [n.decode('utf-8') for n in f['node_names'][:]]
        
        # 2) Extrair tracks: shape (1, 2, num_nodes, num_frames)
        #    Index [0] reduz para (2, num_nodes, num_frames)
        tracks = f['tracks'][0]
        
        # 3) Extrair occupancy para filtrar frames válidos
        occupancy = f['track_occupancy'][:, 0]  # tamanho = num_frames
        
        # 4) Montar colunas do DataFrame: “frame” + (node_x, node_y, node_z) para cada nó
        columns = ["frame"] + [f"{node}_{axis}" 
                               for node in node_names 
                               for axis in ('x', 'y', 'z')]
        data = []
        
        num_frames = tracks.shape[2]  # terceira dimensão = número de frames
        for frame in range(num_frames):
            if occupancy[frame] == 1:
                row = [frame]
                for idx_node, node in enumerate(node_names):
                    x = tracks[0, idx_node, frame]
                    y = tracks[1, idx_node, frame]
                    # Se for NaN, substituímos por 0.0
                    if np.isnan(x) or np.isnan(y):
                        x, y = 0.0, 0.0
                    z = 0.0
                    row.extend([x, y, z])
                data.append(row)
        
        df = pd.DataFrame(data, columns=columns)
        df.to_csv(output_csv_path, index=False)
        print(f"Arquivo salvo em: {output_csv_path}")
        return df

if __name__ == "__main__":
    # Exemplo de uso: ajuste os caminhos conforme seu projeto
    convert_h5_to_csv('H5 test.h5', 'H5_test_mujoco.csv')
    convert_h5_to_csv('labels_Drosoph1.h5', 'labels_Drosoph1_mujoco.csv')
