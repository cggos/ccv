import os
import onnx
import onnx_graphsurgeon as gs
import numpy as np

CG_DM_ROOT = os.getenv("CG_DM_ROOT")

sp_model_root = f"{CG_DM_ROOT}/dataset_ml/superpoint"
model_path = f"{sp_model_root}/super_point.onnx"
output_path = f"{sp_model_root}/super_point_silu.onnx"

# Load model
graph = gs.import_onnx(onnx.load(model_path))
graph.fold_constants().cleanup()  # 提前规整常量

# === 第一阶段：收集待替换节点 ===
nodes_to_replace = []
for node in graph.nodes:
    if node.op == "Relu":
        nodes_to_replace.append(node)
print(f"Found {len(nodes_to_replace)} ReLU nodes to replace")

# === 第二阶段：逐个替换（只改属性，不断开连接）===
for node in nodes_to_replace:
    # 创建新的 LeakyRelu 节点，复用输入输出
    new_node = gs.Node(
        op="LeakyRelu",
        name=f"{node.name}_LeakyReLU",
        inputs=node.inputs,
        outputs=node.outputs,
        attrs={"alpha": 0.1}  # 控制负斜率
    )
    # ⚠️ 关键：不能立即 remove！否则 outputs 失去生产者
    graph.nodes.append(new_node)

# === 第三阶段：安全删除原节点 ===
for node in nodes_to_replace:
    graph.nodes.remove(node)
# === 最终清理：重建拓扑结构 ===
graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), output_path)

