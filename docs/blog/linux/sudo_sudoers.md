---
tags:
    - sudo
    - sudoers
    - nopasswd
    - visudo
---

# sudo hello
Sudo is acronym for **SuperUser DO**
Is allow user access restricted files and operations without logging as `root` user.

sudo is a tool install by default on ubuntu system


## sudoers
`/etc/sudoers` file contain sudo rule and configurations

```
[username] [any-hostname]=([run-as-username]:[run-as-groupname]) [commands-allowed]
```

!!! tip group permission
    Add rule to group `sudo` and not to specific user
     

### edit
To edit `sudoers` file we use the `visudo` command, The command validate the file before saving, 

!!! tip "visudo prompt"
    - e: edit
     


### nopasswd
To grant the permission to run commands as root users without the need of entering the password, set the following line `sudoers` file. 

```
%sudo   ALL=(ALL:ALL) NOPASSWD: apt update
```

## demo 
Add rule for user that assign to `sudo` group to run `apt update` without password

```bash title="before"
sudo -l
Matching Defaults entries for user on lap2:
    env_reset, mail_badpass,
    secure_path=/usr/local/sbin\:/usr/local/bin\:/usr/sbin\:/usr/bin\:/sbin\:/bin\:/snap/bin, use_pty

User user may run the following commands on lap2:
    (ALL : ALL) ALL
```

### edit sudoers
mk-!!! tip
     Command must be in full path
     ```bash
     which apt
     /usr/bin/apt
     ```

```bash
sudo visudo
```

```bash
# Allow members of group sudo to execute any command
%sudo   ALL=(ALL:ALL) ALL
%sudo   ALL=(ALL:ALL) NOPASSWD: /usr/bin/apt
```

### check user privilege
```bash
sudo -l
Matching Defaults entries for user on lap2:
    env_reset, mail_badpass,
    secure_path=/usr/local/sbin\:/usr/local/bin\:/usr/sbin\:/usr/bin\:/sbin\:/bin\:/snap/bin, use_pty

User user may run the following commands on lap2:
    (ALL : ALL) ALL
    (ALL : ALL) NOPASSWD: /usr/bin/apt
```

### Test
- kill current user session
- run apt update **the command sitl need to run with sudo**
  
```
sudo -k
sudo apt update
```

---

## demo II (using /etc/sudoers.d)
The last line in `sudoers` file `@includedir /etc/sudoers.d` run rules defined in this folder

echo "%sudo   ALL=(ALL:ALL) NOPASSWD: /usr/bin/apt" | sudo tee /etc/sudoers.d/apt


!!! warning 
    the visudo command is the recommended way
    to update sudoers content, since it protects against many failure modes
     
---

## demo
Add rule for user that assign to `sudo` group to `mount` tmpfs without switch to `root` and root password

run `visudo` to edit `/etc/sudoers` file

!!! tip "visudo tips"
    Command if visudo found error in `sudoers` file
    - e: edit
     

```
%sudo	ALL=(ALL:ALL) NOPASSWD: /usr/bin/mount -t tmpfs -o size=10m tmpfs /mnt/ramfs 

```

- 
```bash title="list user privileges"
sudo -l
#
...

User user may run the following commands on lap2:
    (ALL : ALL) ALL
    (ALL : ALL) NOPASSWD: /usr/bin/mount -t tmpfs -o size\=10m tmpfs /mnt/ramfs


```

```bash title="kill current sudo session"
sudo -k
```