---
tags:
    - klv
    - misb
    - mpeg-ts
    - gst
---

Using gstreamer to mix klv metadata into video using mpeg-ts



## Standards
### MISB
[MISB ST 601](https://kubic-nsg-standards-nsgreg-nsgreg-files-6lxvt.s3.us-east-1.amazonaws.com/doc/Document/ST0601.19.pdf?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVXR7TTKDQUGW36FA%2F20240410%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20240410T040829Z&X-Amz-Expires=7200&X-Amz-SignedHeaders=host&response-cache-control=7200&response-content-disposition=inline&response-content-type=application%2Fpdf&X-Amz-Signature=560b0a3bfc6289d3968664a1d71c1fda336d1db9a5f022d75a8d76498e496b84) defines the Unmanned Air System (UAS) Datalink Local Set
The UAS Datalink LS is a bandwidth-efficient, **extensible Key-Length-Value (KLV) metadata**

---

### STANAG 4609
NATO Digital Motion Imaging [more](https://impleotv.com/situational-awareness-and-stanag-4609/)

---

### Code

#### klvdata
[klvdata](https://github.com/paretech/klvdata/tree/master) is a Python library for parsing and constructing Key Length Value (KLV) formatted binary streams.