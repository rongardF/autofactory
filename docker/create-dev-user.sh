#!/usr/bin/env bash
# Reuse Ubuntu's default account when it occupies the requested UID.
set -euo pipefail
name=${1:?username required}
uid=${2:?UID required}
gid=${3:?GID required}
if getent passwd "$name" >/dev/null; then
    [[ $(id -u "$name") == "$uid" && $(id -g "$name") == "$gid" ]] || {
        echo "Existing $name account has unexpected UID/GID" >&2; exit 1;
    }
else
    if getent group "$gid" >/dev/null; then
        group_name=$(getent group "$gid" | cut -d: -f1)
        if [[ $group_name == ubuntu ]]; then groupmod --new-name "$name" ubuntu; fi
    else
        groupadd --gid "$gid" "$name"
    fi
    if getent passwd "$uid" >/dev/null; then
        owner=$(getent passwd "$uid" | cut -d: -f1)
        [[ $owner == ubuntu ]] || {
            echo "UID $uid belongs to $owner; choose another USER_UID" >&2; exit 1;
        }
        usermod --login "$name" --home "/home/$name" --move-home --gid "$gid" --shell /bin/bash ubuntu
    else
        useradd --uid "$uid" --gid "$gid" --create-home --shell /bin/bash "$name"
    fi
fi
printf '%s ALL=(root) NOPASSWD:ALL\n' "$name" > "/etc/sudoers.d/$name"
chmod 0440 "/etc/sudoers.d/$name"
visudo -cf "/etc/sudoers.d/$name"
