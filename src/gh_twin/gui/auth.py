import os
import json
import hashlib

USER_DB = 'users.txt'


def hash_password(password: str) -> str:
    return hashlib.sha256(password.encode()).hexdigest()


def load_users():

    if not os.path.exists(USER_DB):

        with open(USER_DB, 'w') as f:
            json.dump({}, f)

    with open(USER_DB, 'r') as f:
        return json.load(f)


def authenticate(username: str, password: str):

    users = load_users()

    if username not in users:
        return False

    return users[username] == hash_password(password)