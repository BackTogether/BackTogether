"""Who is in the drone video? Finds faces and compares them with the people database.

The database is a folder of reference photos, one folder per person (not in git):

    drone/identify/people/maria/1.jpg
    drone/identify/people/maria/2.jpg
    drone/identify/people/juan/1.jpg

Models (open source, offline, CPU): OpenCV YuNet finds the faces and OpenCV SFace turns each one
into a vector; two photos of the same person give vectors that point the same way.
"""
