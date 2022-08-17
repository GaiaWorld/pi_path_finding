# Unity 导航网格 导出插件

格式可以任意转换，下面是一种导出法

``` txt
v
0.0 0.0 0.0
5.0 0.0 0.0
0.0 7.0 0.0
5.0 7.0 0.0
10.0 7.0 0.0
5.0 12.0 0.0
10.0 12.0 0.0
26.0 12.0 0.0
10.0 21.0 0.0
f
0 2 1
1 2 3
1 3 4
3 5 4
4 5 6
4 6 7
6 8 7

```

## 代码

``` C#

StreamWriter streamWriter = new StreamWriter(path);

// 顶点坐标
streamWriter.WriteLine("v");

List<Vector3> vertexs = new List<Vector3>();

Dictionary<int, int> vertexIndex = new Dictionary<int, int>();

for (int i = 0; i < navMeshTriangulation.vertices.Length; i++)
{
    bool newVert = true;
    for (int ii = 0; ii < vertexs.Count; ii++)
    {
        if ((vertexs[ii] - navMeshTriangulation.vertices[i]).magnitude < 0.0001)
        {
            newVert = false;
            vertexIndex.Add(i, ii);
            break;
        }
    }
    if (newVert)
    {
        vertexIndex.Add(i, vertexs.Count);
        vertexs.Add(navMeshTriangulation.vertices[i]);
        streamWriter.WriteLine(navMeshTriangulation.vertices[i].x + " " + navMeshTriangulation.vertices[i].y + " " + navMeshTriangulation.vertices[i].z);
    }
}

List<List<int>> polygons = new List<List<int>>();
List<int> polygon_types = new List<int>();
for (int i = 0; i < navMeshTriangulation.indices.Length;)
{
    bool merge = false;
    foreach (var polygon in polygons)
    {
        if (polygon.Contains(navMeshTriangulation.indices[i]) && polygon.Contains(navMeshTriangulation.indices[i + 1]))
        {
            polygon.Add(navMeshTriangulation.indices[i + 2]);
            merge = true;
            break;
        }
        if (polygon.Contains(navMeshTriangulation.indices[i + 1]) && polygon.Contains(navMeshTriangulation.indices[i + 2]))
        {
            polygon.Add(navMeshTriangulation.indices[i]);
            merge = true;
            break;
        }
        if (polygon.Contains(navMeshTriangulation.indices[i]) && polygon.Contains(navMeshTriangulation.indices[i + 2]))
        {
            polygon.Add(navMeshTriangulation.indices[i + 1]);
            merge = true;
            break;
        }
    }
    if(!merge)
    {
        List<int> temp = new List<int>();
        temp.Add(navMeshTriangulation.indices[i]);
        temp.Add(navMeshTriangulation.indices[i + 1]);
        temp.Add(navMeshTriangulation.indices[i + 2]);
        polygons.Add(temp);
        polygon_types.Add(navMeshTriangulation.areas[i/3]);
    }
    i = i + 3;
}

// 面
streamWriter.WriteLine("f");
for (int i = 0; i < polygons.Count; i++)
{
    string line = "";
    for(int ii = 0; ii < polygons[i].Count; ii++)
    {
        int index = vertexIndex[polygons[i][ii]];
        if (ii != 0)
            line += " " + index.ToString();
        else
            line += index.ToString();
    }
    streamWriter.WriteLine(line);
}

// 面积，就是每个三角形的代价 cost
streamWriter.WriteLine("a");
for (int i = 0; i < polygon_types.Count; i++)
{
    streamWriter.WriteLine(navMeshTriangulation.areas[i]);
}

streamWriter.Flush();
streamWriter.Close();

```