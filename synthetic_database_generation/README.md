In order to adapt the synthetic database, this program should be editted depending on the requierements.

1. Generate objects CAD:
```bash
cd synthetic_database_generation
python3 make_shapes.py
```

2. Generate the database through random scenes generated in a PyBullet simulation:
```bash
cd synthetic_database_generation
python3 make_scene.py
```
