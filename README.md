# TurtleBot3 Maze Devriye

Bu proje, TurtleBot3 robotunun bir harita üzerinde **Voronoi Diagram** temelli graf kullanarak en kapsayıcı tüm alanı dolaşmasını ve devriye görevini gerçekleştirmesini amaçlamaktadır. Robot, haritanın tamamını etkili bir şekilde kapsamak için voronoi diagram yaklaşımını ve hareket planlama algoritmalarından faydalanır.

---

## Proje Amacı

Bu proje, **Voronoi Diagram** kullanarak haritalanmış bir ortamda robotun devriye görevini gerçekleştirmesi için bir çözüm sunar. Robot, haritanın tümüne erişmek için oluşturulan grafikteki düğümleri ziyaret eder. Görevin tamamında robotun toplam kat ettiği mesafe ve kapsadığı alan yüzdesi gibi performans ölçütleri hesaplanır. 

Bu amaç doğrultusunda:
1. **Voronoi diagram** ve graf oluşturulur.
2. Robotun hedefe ulaşması için **graph mantıklı bir şekilde traverse edilir**.
3. Kapsanan alan ve performans ölçümleri görselleştirilir.

---

## Kurulum ve Gereksinimler

### 2. TUW Multi Robot Paketi Kurulumu
(https://github.com/tuw-robotics/tuw_multi_robot)

Voronoi diagramı ile graf oluşturmak için gerekli olan `tuw_multi_robot` paketini kurmak için aşağıdaki adımları takip edin: 
```bash
sudo apt install libdxflib-dev
export ROS_VERSION=noetic   # for Ubuntu 20.04
sudo apt install ros-$ROS_VERSION-map-server
sudo apt install ros-$ROS_VERSION-stage-ros
export MRRP_DIR=$HOME/projects/catkin/tuw_multi_robot/
mkdir -p $MRRP_DIR/src
cd $MRRP_DIR/src
git clone --branch $ROS_VERSION git@github.com:tuw-robotics/tuw_multi_robot.git 
git clone --branch $ROS_VERSION git@github.com:tuw-robotics/tuw_geometry.git 
git clone git@github.com:tuw-robotics/tuw_msgs.git 
```

Derleme sırasında hata alırsanız tuw_multi_robot'un master branchini kurun.

### 3. Özel Voronoi Node'u Eklenmesi

tuw_multi_robot paketindeki voronoi_graph_node.cpp dosyasını projede verilen yeni voronoi_graph_node.cpp dosyasıyla değiştirin:
```
cd ~/robotlar_ws/src/tuw_multi_robot/tuw_voronoi_graph/src
cp /path/to/custom_voronoi_graph_node.cpp voronoi_graph_node.cpp
cd ~/robotlar_ws
catkin_make
source devel/setup.bash
```
micromousemaze ortamına göre değişiklikler yapılmıştır:

```
void VoronoiGeneratorNode::publishSegments()
{
graph.origin.position.x = -10.0; // SOLVE MAZE ICIN -10
graph.origin.position.y = -10.0; // SOLVE MAZE ICIN -10

pos.x = path[i][0] * resolution_ + graph.origin.position.x; // originler eklendi
pos.y = path[i][1] * resolution_ + graph.origin.position.y; // originler eklendi

```

### 4. Projenin Çalıştırılması
**Simülasyon ve Harita**

Aşağıdaki adımları sırasıyla terminalde çalıştırarak simülasyonu ve haritayı başlatın:
##### Adım 1: Labirenti başlatın
```
roslaunch micromouse_maze micromouse_maze4.launch
```
##### Adım 2: Harita sunucusunu başlatın
```
rosrun map_server map_server $(rospack find micromouse_maze)/maps/map.yaml
```
### 5. TurtleBot3 Navigasyon
##### Adım 3: AMCL (Adaptive Monte Carlo Localization)
```
roslaunch turtlebot3_navigation amcl.launch
```
##### Adım 4: Move Base
Yeşil path move base :
```
roslaunch turtlebot3_navigation move_base.launch
```
<p>
<img src="turtlebot3_maze_devriye/imgs/voronili_path.png" width="874"/>
</p>

### 6. Voronoi 
##### Adım 5: Voronoi Diagramı Oluşturun
Bu kod, bir harita (OccupancyGrid) girdisi alarak bu haritanın üzerinde Voronoi grafiği oluşturur. Voronoi grafiği, haritadaki serbest alanlar boyunca en uygun yolların bir ağ yapısı olarak temsil edilmesini sağlar. Oluşturulan grafik, ROS topicleri üzerinden yayınlanarak diğer sistemlerle kullanılabilir hale gelir. Özellikle devriye görevi için mantıklı bir kullanımdır. 

Yayınlanan segmentlere bakmak için:
```
rostopic echo /segments
```
Çalıştırmak için:
```
rosrun tuw_voronoi_graph tuw_voronoi_graph_node
```
<p>
<img src="turtlebot3_maze_devriye/imgs/tuw_voronoi_graph_voronoi_graph_node.png" width="874"/>
</p>

Map:
<p>
<img src="turtlebot3_maze_devriye/turtlebot3_devriye/voronoi_graph/map.png" width="374"/>
</p>

##### Adım 6: Segmentlerin Görselleştirilmesi

voronoi_segments node'u ile voronoi_segments_marker topici üzerinden yayınlanan marker tipindeki mesaj rvizde dinlenerek voronoi diagramının segmentleri görselleştirilir.
```
rosrun turtlebot3_devriye voronoi_segments
```
<p>
<img src="turtlebot3_maze_devriye/imgs/voronoili_map.png" width="874"/>
</p>

### 7. Devriye Görevi
##### Adım 7: Devriye node'unu çalıştırın
```
rosrun turtlebot3_devriye devriye
```
- Devriye Videosu:
<p>
<img src="turtlebot3_maze_devriye/imgs/gezinim.gif" width="1000"/>
</p>
- Devriye sırası ve devriyenin tamamlanması:
<p>
<img src="turtlebot3_maze_devriye/imgs/devriye_1.png" width="874"/>
</p>

<p>
<img src="turtlebot3_maze_devriye/imgs/finished.png" width="874"/>
</p>

