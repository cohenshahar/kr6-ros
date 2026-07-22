# kr6-ros — מערכת ה-VLA על KR6, ב-ROS2

המרת מערכת הניסוי (MuJoCo scene_v2 + ACT/SmolVLA + executor) לארכיטקטורת ROS2 Humble
של שלושה נודים: סימולציה / מדיניות / ביצוע. פייתון בלבד (rclpy).
תוכנית מלאה: [PLAN_ROS.md](PLAN_ROS.md) · הערות שחר: [FEEDBACK.md](FEEDBACK.md)

## סטטוס שערים
| שער | תיאור | סטטוס |
|---|---|---|
| R0 | שלד: 5 חבילות נבנות, 3 נודים עולים | ✅ 22/07 |
| R1 | sim_node טוען scene_v2, reset/apply, וידאו | ✅ 22/07 |
| R2 | אפיזודת ACT מקצה-לקצה ב-lockstep | 🔨 בעבודה |
| R3 | שחזור 23/24 של ACT C0 מול ה-JSON הקיים | ⬜ |
| R4 | SmolVLA בפרמטר — שחזור 19/24 | ⬜ |
| R5 | מצב זרימה חופשית + מדידות | ⬜ |

## בנייה והרצה
```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch kr6_eval kr6_system.launch.py
```
