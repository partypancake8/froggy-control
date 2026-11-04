// cadkit view: the one standard local three.js CAD viewer (served by cadkit/viewer/server.py).
// Loads every file of /models.json (.glb, .stl, .ply; mm, Z up), one tree module per file or directory group
// (or the sheet.json module tree), with review or fitted camera buttons, section planes, explode, picking, a
// movable group with pose save, live reload, and one undo/redo history over every committed state change.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {PLYLoader} from 'three/addons/loaders/PLYLoader.js';
const $=id=>document.getElementById(id);
const log=(level,msg)=>window.__log&&window.__log(level,msg);
const D2R=Math.PI/180,R2D=180/Math.PI;
THREE.Object3D.DEFAULT_UP.set(0,0,1);

// ---- renderer, cameras, lights ----
const host=$('view'),canvas=$('c');
// WebGL when the browser has it; otherwise (headless Chrome with --disable-gpu) a software z-buffer preview on
// a 2D canvas that draws the same scene state (camera, visibility, explode, clip, highlight).
function webglOK(){try{const c=document.createElement('canvas');return !!(c.getContext('webgl2')||c.getContext('webgl'));}catch(e){return false;}}
let renderer=null;
if(webglOK()){try{renderer=new THREE.WebGLRenderer({canvas,antialias:true});}catch(e){renderer=null;}}
if(renderer){renderer.setPixelRatio(devicePixelRatio);renderer.localClippingEnabled=true;renderer.autoClear=false;}
const SOFT=!renderer;
const scene=new THREE.Scene();scene.background=new THREE.Color('#e5e7eb');
const FOV=30,OB=200;                                   // ortho base half-height (mm at zoom 1)
const persp=new THREE.PerspectiveCamera(FOV,1,1,100000);
const ortho=new THREE.OrthographicCamera(-OB,OB,OB,-OB,1,100000);
for(const c of [persp,ortho]){c.up.set(0,0,1);scene.add(c);}
let cam=ortho;
scene.add(new THREE.HemisphereLight(0xffffff,0x78818c,0.85));
const key=new THREE.DirectionalLight(0xffffff,1.0);key.position.set(-0.5,0.8,1);cam.add(key);
const controls=new OrbitControls(cam,canvas);
Object.assign(controls,{enableDamping:!SOFT,zoomToCursor:true,screenSpacePanning:true,rotateSpeed:0.8,minDistance:1});
// OrbitControls 0.160 reports a change on every update() for an orthographic camera, so movement is detected here
let dirty=true,renders=0,ctrlTrue=0;
const lastC={cam:null,p:new THREE.Vector3(),q:new THREE.Quaternion(),z:0,t:new THREE.Vector3()};
function camMoved(){
  const m=lastC.cam!==cam||lastC.p.distanceToSquared(cam.position)>1e-8||8*(1-Math.abs(lastC.q.dot(cam.quaternion)))>1e-9||
    Math.abs(lastC.z-cam.zoom)>1e-9||lastC.t.distanceToSquared(controls.target)>1e-8;
  if(m){lastC.cam=cam;lastC.p.copy(cam.position);lastC.q.copy(cam.quaternion);lastC.z=cam.zoom;lastC.t.copy(controls.target);}
  return m;
}
// ground grid at Z 0 (20 mm cells, grows with the model) and axes at the origin
const GRID={size:600,cell:20};let grid=null,showGrid=false;
function setGrid(size){
  if(grid&&GRID.size===size)return;GRID.size=size;
  if(grid){scene.remove(grid);grid.geometry.dispose();}
  grid=new THREE.GridHelper(size,Math.round(size/GRID.cell),0x64748b,0xabb2bd);grid.rotation.x=Math.PI/2;scene.add(grid);dirty=true;
  grid.visible=showGrid;
}
setGrid(600);
function setGridVisible(visible){
  showGrid=visible;grid.visible=visible;
  $('grid').classList.toggle('on',visible);$('grid').setAttribute('aria-pressed',String(visible));
  $('grid').title=visible?'Hide ground grid':'Show ground grid';dirty=true;
}
$('grid').onclick=()=>{setGridVisible(!showGrid);commit();};
const axes=new THREE.AxesHelper(60);scene.add(axes);
// corner triad
const tScene=new THREE.Scene(),tCam=new THREE.OrthographicCamera(-1.6,1.6,1.6,-1.6,0.1,10);
tCam.up.set(0,0,1);tScene.add(new THREE.AxesHelper(1));
for(const [t,p,c] of [['X',[1.3,0,0],'#b42318'],['Y',[0,1.3,0],'#166534'],['Z',[0,0,1.3],'#1d4ed8']]){
  const cv=document.createElement('canvas');cv.width=cv.height=64;const g=cv.getContext('2d');
  g.fillStyle=c;g.font='bold 48px system-ui';g.textAlign='center';g.textBaseline='middle';g.fillText(t,32,34);
  const s=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(cv),depthTest:false}));
  s.position.set(...p);s.scale.setScalar(0.55);tScene.add(s);
}
function resize(){
  const w=host.clientWidth,h=host.clientHeight;if(!w||!h)return;
  if(renderer)renderer.setSize(w,h,false);const a=w/h;
  persp.aspect=a;persp.updateProjectionMatrix();
  ortho.left=-OB*a;ortho.right=OB*a;ortho.top=OB;ortho.bottom=-OB;ortho.updateProjectionMatrix();dirty=true;
}
new ResizeObserver(resize).observe(host);resize();

// ---- movable group (--movable): gizmo, pose fields, nudges, pose save ----
let M=null;                         // /models.json manifest
let mask=null,tc=null;              // the movable wrapper group (persists over reloads) and its gizmo
const loadedPose={p:new THREE.Vector3(),q:new THREE.Quaternion(),s:new THREE.Vector3(1,1,1)};
let savedPose=null,headMode=false;

// ---- camera buttons: review cameras from /views.json, or iso/front/right/back/top/bottom fitted to the model ----
let VIEWS=[],fitted=false,activeView=null;
const viewHidden=new Set();     // leaf keys hidden by the current view's `hide` list
function setCam(c){
  if(c===cam)return;
  const t=controls.target.clone(),dir=cam.position.clone().sub(t);
  const half=cam.isOrthographicCamera?OB/cam.zoom:dir.length()*Math.tan(cam.fov*Math.PI/360);
  dir.normalize();
  if(c.isOrthographicCamera){c.zoom=OB/half;c.position.copy(t).addScaledVector(dir,Math.max(2000,half*6,radius*4));}
  else c.position.copy(t).addScaledVector(dir,half/Math.tan(c.fov*Math.PI/360));
  c.up.copy(cam.up);c.updateProjectionMatrix();c.lookAt(t);
  useCam(c);controls.update();
}
function useCam(c){
  if(c!==cam){cam.remove(key);c.add(key);cam=c;controls.object=c;if(tc)tc.camera=c;}
  $('proj').textContent=c.isOrthographicCamera?'Ortho':'Perspective';dirty=true;
}
// ---- camera view (kind "camera"): the true lens image, not a perspective stand-in. A cube map rendered at the
// lens point (eye) holds the whole scene exactly as the tree shows it; only the view's fisheye.observer parts (the
// camera's own board and lens, which a lens cannot see) are left out of that one render, the way lib/camsim.py
// leaves them out of its rays. Every output pixel is a sensor pixel of the view's lens model: p = pixel offset
// from the frame centre (sensor_px, image right/up), theta = |p| / f (equidistant, f in px/rad), direction =
// right*sin(theta)cos(phi) + up*sin(theta)sin(phi) + axis*cos(theta), black beyond theta_max (the image circle).
// The frame keeps the sensor aspect, centred in the view. Orbit is off (the lens fixes the viewpoint); Escape or
// any other view button leaves it. Picking and hover cast the same per-pixel rays from the lens.
let camMode=null,projBefore=null,viewBefore=null;   // the active camera view, the camera and view to go back to
const FISH={rt:null,cube:null,scene:null,cam:null,mat:null,rect:null};
function fishInit(fe){
  if(FISH.mat)return;
  const gl2=renderer.capabilities.isWebGL2;      // half float faces: no banding in the dark drum
  FISH.rt=new THREE.WebGLCubeRenderTarget(1024,{type:gl2?THREE.HalfFloatType:THREE.UnsignedByteType,generateMipmaps:false,
    minFilter:THREE.LinearFilter,magFilter:THREE.LinearFilter});
  FISH.cube=new THREE.CubeCamera(fe.near_mm||0.5,5000,FISH.rt);
  FISH.mat=new THREE.ShaderMaterial({
    uniforms:{env:{value:FISH.rt.texture},R:{value:new THREE.Matrix3()},sensor:{value:new THREE.Vector2(4,3)},f:{value:1},thMax:{value:Math.PI},px:{value:1}},
    vertexShader:'varying vec2 vUv;void main(){vUv=uv;gl_Position=vec4(position.xy,0.0,1.0);}',
    // 2 x 2 sub-pixel rays per screen pixel (px = sensor pixels per screen pixel), each black beyond the circle
    fragmentShader:`uniform samplerCube env;uniform mat3 R;uniform vec2 sensor;uniform float f;uniform float thMax;uniform float px;varying vec2 vUv;
vec3 ray(vec2 p){float r=length(p);float th=r/f;if(th>thMax)return vec3(0.0);
  vec2 q=r>1e-6?p/r:vec2(0.0);return textureCube(env,R*vec3(q*sin(th),cos(th))).rgb;}
void main(){
  vec2 p=(vUv-0.5)*sensor;float h=0.25*px;
  gl_FragColor=vec4(0.25*(ray(p+vec2(-h,-h))+ray(p+vec2(h,-h))+ray(p+vec2(-h,h))+ray(p+vec2(h,h))),1.0);
  #include <colorspace_fragment>
}`,depthTest:false,depthWrite:false});
  FISH.scene=new THREE.Scene();FISH.scene.add(new THREE.Mesh(new THREE.PlaneGeometry(2,2),FISH.mat));
  FISH.cam=new THREE.OrthographicCamera(-1,1,1,-1,0,1);
}
function fishRect(){
  const W=host.clientWidth,H=host.clientHeight,g=12,top=44,s=camMode.fisheye.sensor_px,a=s[0]/s[1];
  const w=Math.max(1,Math.min(W-2*g,(H-top-g)*a)),h=w/a;
  return {x:Math.round((W-w)/2),y:Math.round(top+(H-top-g-h)/2),w:Math.round(w),h:Math.round(h)};
}
const observerObjs=()=>{const o=(camMode&&camMode.fisheye&&camMode.fisheye.observer)||[],out=[];
  for(const [,r] of nodes)if(r.leaf&&o.includes(r.leaf.component))out.push(r.obj);return out;};
function fishRender(){
  const v=camMode,fe=v.fisheye;fishInit(fe);
  const r=fe.right,u=fe.up,a=fe.axis,U=FISH.mat.uniforms;
  U.R.value.set(r[0],u[0],a[0],r[1],u[1],a[1],r[2],u[2],a[2]);
  U.sensor.value.set(fe.sensor_px[0],fe.sensor_px[1]);U.f.value=fe.f_px_per_rad;U.thMax.value=fe.theta_max_rad;
  FISH.cube.position.fromArray(v.eye);FISH.cube.updateMatrixWorld(true);
  // viewer helpers (axes, grid, gizmo) and the observer's own body are not in the lens image; the tree is untouched
  const off=[axes,grid,tc,...observerObjs()].filter(o=>o&&o.visible);
  for(const o of off)o.visible=false;
  const ac=renderer.autoClear;renderer.autoClear=true;
  try{FISH.cube.update(renderer,scene);}finally{renderer.autoClear=ac;for(const o of off)o.visible=true;}
  renderer.setRenderTarget(null);
  const W=host.clientWidth,H=host.clientHeight;renderer.setScissorTest(false);renderer.setViewport(0,0,W,H);
  renderer.setClearColor(0x0d1117,1);renderer.clear();
  const R=fishRect();FISH.rect=R;U.px.value=fe.sensor_px[0]/R.w;renderer.setViewport(R.x,H-R.y-R.h,R.w,R.h);renderer.render(FISH.scene,FISH.cam);
}
function fishDir(ev){
  const R=FISH.rect;if(!R)return null;const cr=canvas.getBoundingClientRect(),fe=camMode.fisheye;
  const u=(ev.clientX-cr.left-R.x)/R.w,w=1-(ev.clientY-cr.top-R.y)/R.h;if(u<0||u>1||w<0||w>1)return null;
  const x=(u-0.5)*fe.sensor_px[0],y=(w-0.5)*fe.sensor_px[1],r=Math.hypot(x,y),th=r/fe.f_px_per_rad;if(th>fe.theta_max_rad)return null;
  const s=r>1e-9?Math.sin(th)/r:0;
  return new THREE.Vector3().fromArray(fe.right).multiplyScalar(x*s).addScaledVector(new THREE.Vector3().fromArray(fe.up),y*s)
    .addScaledVector(new THREE.Vector3().fromArray(fe.axis),Math.cos(th)).normalize();
}
const fishOn=()=>!!(camMode&&camMode.fisheye&&!SOFT);
function enterCamMode(v){
  if(!camMode){projBefore=cam;viewBefore=activeView;}
  camMode=v;persp.fov=FOV;persp.updateProjectionMatrix();useCam(persp);camUI();
}
function exitCamMode(){
  if(!camMode)return;
  camMode=null;
  if(projBefore&&projBefore!==cam)setCam(projBefore);
  projBefore=null;camUI();
}
function camLabel(){
  if(!camMode){$('camnote-t').textContent='';return;}
  let n=0;for(const [k,r] of nodes)if(r.leaf&&!leafShown(k))n++;
  const extra=[];if(+$('explode').value>0)extra.push('explode on');if(clipPlanes.length)extra.push('section on');
  if(SOFT&&camMode.fisheye)extra.push('no WebGL: perspective preview, not the lens');
  $('camnote-t').textContent=(camMode.note||camMode.caption||camMode.name)+', '+(n?n+' part'+(n===1?'':'s')+' hidden':'nothing hidden')+(extra.length?', '+extra.join(', '):'');
}
function camUI(){
  const v=camMode;$('proj').disabled=!!v;$('proj').title=v?'The camera view is the lens projection':'Orthographic matches the review packet PNGs';
  $('camnote').hidden=!v;controls.enabled=!fishOn();$('triad-label').hidden=fishOn();camLabel();
  if(!v)FISH.rect=null;
  dirty=true;
}
function applyCameraView(v){
  enterCamMode(v);
  const eye=new THREE.Vector3(...v.eye),f=v.focal?new THREE.Vector3(...v.focal):eye.clone().addScaledVector(new THREE.Vector3(...v.axis).normalize(),v.scale||100);
  tween=null;cam.up.set(0,0,1);controls.target.copy(f);cam.position.copy(eye);cam.zoom=1;
  cam.updateProjectionMatrix();cam.lookAt(f);controls.update();
}
function applyView(v){
  if(v.kind==='camera')applyCameraView(v);else{exitCamMode();applyFramedView(v);}
  // the view's hide list (module:<name> or a regex over component names), undone by the next view
  for(const k of viewHidden){if(nodes.has(k)&&!leafShown(k))setVis(k,true,false);}
  viewHidden.clear();
  // the camera view is a reference of what the lens sees: entering it shows every part
  if(v.kind==='camera')for(const [k,n] of nodes)if(n.leaf)vis.set(k,true);
  for(const [k,n] of nodes){
    if(!n.leaf)continue;
    const hit=(v.hide||[]).some(h=>h.startsWith('module:')?n.leaf.module===h.slice(7):new RegExp(h).test(n.leaf.component));
    if(hit&&leafShown(k)){setVis(k,false,false);viewHidden.add(k);}
  }
  applyVisibility();
  markView(v.name);
}
function applyFramedView(v){
  const eye=new THREE.Vector3(...v.eye),f=new THREE.Vector3(...v.focal),up=new THREE.Vector3(...v.up).normalize();
  // the orbit stays Z-up; a view with another up vector (top: +Y up, looking down -Z) gets its eye nudged a hair
  // toward -up, which gives the same screen orientation and framing
  if(Math.abs(up.z)<0.99)eye.addScaledVector(up,-1e-3*eye.distanceTo(f));
  const dir=eye.clone().sub(f);
  cam.up.set(0,0,1);controls.target.copy(f);
  if(cam.isOrthographicCamera){cam.position.copy(f).addScaledVector(dir.clone().normalize(),Math.max(dir.length(),radius*4));cam.zoom=OB/v.scale;}
  else cam.position.copy(f).addScaledVector(dir.normalize(),v.scale/Math.tan(FOV*Math.PI/360));
  cam.updateProjectionMatrix();cam.lookAt(f);controls.update();
}
function markView(n){activeView=n;document.querySelectorAll('#views button').forEach(b=>b.classList.toggle('on',b.dataset.v===n));dirty=true;}
const FITTED=[['iso',[1,-1,0.8],[0,0,1]],['front',[0,-1,0],[0,0,1]],['right',[1,0,0],[0,0,1]],['back',[0,1,0],[0,0,1]],
  ['top',[0,0,1],[0,1,0]],['bottom',[0,0,-1],[0,-1,0]]];
function fittedViews(){
  const c=bbox.getCenter(new THREE.Vector3()),r=Math.max(1,radius);
  return FITTED.map(([name,d,up])=>{const n=new THREE.Vector3(...d).normalize();
    return {name,caption:name+' view fitted to the model',eye:c.clone().addScaledVector(n,r*4).toArray(),focal:c.toArray(),up,scale:r*1.08};});
}
function renderViewButtons(){
  const box=$('views');box.innerHTML='';
  for(const v of VIEWS){
    const b=document.createElement('button');b.dataset.v=v.name;b.textContent=v.label||v.name;b.title=v.caption||v.name;
    b.onclick=()=>camAction(()=>applyView(v));box.append(b);
  }
  markView(activeView);
}
async function loadViews(){
  let j={};try{j=await (await fetch('/views.json')).json();}catch(e){console.warn('views.json',e);}
  VIEWS=j.views||[];fitted=!!j.fitted||!VIEWS.length;
  renderViewButtons();
}
function fitAll(){exitCamMode();const b=new THREE.Box3();for(const m of meshList)if(effVisible(m))b.expandByObject(m);if(!b.isEmpty())focusBox(b,false);markView(null);}
$('proj').onclick=()=>{if(!camMode)camAction(()=>setCam(cam===ortho?persp:ortho));};
$('fit').onclick=()=>camAction(fitAll);

// ---- model, tree, visibility ----
const gltfLoader=new GLTFLoader(),stlLoader=new STLLoader(),plyLoader=new PLYLoader();
let modelRoot=null;                 // THREE.Group of module groups
let meshByNode=new Map();           // "<file index>:<node name>" -> Mesh
let meshList=[];
let tree=null;
const nodes=new Map();              // key -> {obj, leaf?, el, cb, path, count}
const vis=new Map();                // key -> bool (survives reloads, by tree path)
const openState=new Map();          // key -> bool
const modules=[];                   // {key, group, centroid}
const explodeParts=[];              // fitted overlays: independent meshes, with their assembled transforms
let topKeys=[];
const keyOf=path=>path.join(' > ');
const srgb=c=>c<=0.04045?c/12.92:Math.pow((c+0.055)/1.055,2.4);
const clipPlanes=[];
const planeZ=new THREE.Plane(new THREE.Vector3(0,0,-1),0),planeX=new THREE.Plane(new THREE.Vector3(-1,0,0),0);
const HL=new THREE.Color('#1f6feb');
const PALETTE=[0x7a9cc6,0xc6937a,0x8fb88f,0xb88fb0,0xc9b67a,0x7ab8b8,0xb87a7a,0x9a9ac9,0xa8b86f,0xb8a08f];
const groupOf=i=>M.groups.find(g=>g.files.includes(i));
const inMask=g=>!!(g&&mask&&(g.movable||g.second));

function prepMesh(m,glb){
  const mats=Array.isArray(m.material)?m.material:[m.material];
  for(const mat of mats){
    // glTF factors are linear but trimesh wrote the sRGB hex values: convert so the colours match sheet.json
    if(glb)mat.color&&mat.color.convertSRGBToLinear();
    mat.side=THREE.DoubleSide;mat.clippingPlanes=clipPlanes;mat.clipShadows=true;
    mat.userData.emissive0=mat.emissive?mat.emissive.clone():null;
  }
  const col=m.geometry.attributes.color;
  if(glb&&col){
    const a=col.array,n=col.itemSize,norm=col.normalized,div=a instanceof Uint8Array?255:a instanceof Uint16Array?65535:1;
    const f=new Float32Array(a.length);
    for(let i=0;i<a.length;i++){const v=norm||div===1?a[i]/div:a[i];f[i]=(i%n)===3?v:srgb(v);}
    m.geometry.setAttribute('color',new THREE.BufferAttribute(f,n));
  }
}
function meshMaterial(f,geo){
  const g=groupOf(f.i);
  if(f.color){
    const opacity=inMask(g)?+$('op').value:1;
    // Matte CAD display colours lift dark prints enough to reveal their geometry.
    // The manifest retains the actual filament colour.
    const color=new THREE.Color(f.color);
    if(Math.max(color.r,color.g,color.b)<0.04)color.set('#7b8490');
    const material=new THREE.MeshStandardMaterial({color,roughness:1,metalness:0,transparent:opacity<1,opacity,depthWrite:opacity>=1});
    material.userData.printColor=f.color;
    return material;
  }
  if(inMask(g)&&headMode){
    const opacity=+$('op').value;
    return new THREE.MeshStandardMaterial({color:0x8b949e,roughness:0.6,metalness:0.1,transparent:opacity<1,opacity,depthWrite:opacity>=1});
  }
  if(geo.hasAttribute('color'))return new THREE.MeshStandardMaterial({vertexColors:true,roughness:0.85,metalness:0});
  const opaque=!inMask(g)||+$('op').value>=1;
  return new THREE.MeshStandardMaterial({color:PALETTE[f.i%PALETTE.length],roughness:0.6,metalness:0.1,transparent:!opaque,opacity:opaque?1:+$('op').value});
}
const leafKey=n=>(n.file??M.first_glb??0)+':'+n.node;

function assemble(){
  // module > sub-assembly > (Fasteners > joint >) parts, as THREE groups so visibility and explode are per node
  resetExplode();
  if(modelRoot){scene.remove(modelRoot);}
  modelRoot=new THREE.Group();modelRoot.name='model';scene.add(modelRoot);
  if(mask){mask.clear();modelRoot.add(mask);}
  nodes.clear();modules.length=0;topKeys=[];
  const used=new Set();
  function build(n,path,parent,depth){
    const p=path.concat([n.name]);let k=keyOf(p);
    for(let i=2;nodes.has(k);i++)k=keyOf(p)+' #'+i;          // duplicate sibling names stay separate
    if(!n.children){
      const lk=leafKey(n),m=meshByNode.get(lk);if(!m)return [];
      used.add(lk);m.userData.key=k;m.userData.leaf=n;
      if(depth===0){const g=new THREE.Group();g.name=n.name;g.userData.key=k;g.add(m);parent.add(g);nodes.set(k,{obj:g,leaf:n,path:p,count:1,leaves:[k]});topKeys.push(k);}
      else{parent.add(m);nodes.set(k,{obj:m,leaf:n,path:p,leaves:[k]});}
      return [k];
    }
    const g=new THREE.Group();g.name=n.name;g.userData.key=k;parent.add(g);
    const rec={obj:g,path:p,count:0,depth,leaves:[]};nodes.set(k,rec);if(depth===0)topKeys.push(k);
    for(const ch of n.children)for(const l of build(ch,p,g,depth+1))rec.leaves.push(l);
    rec.count=rec.leaves.length;return rec.leaves;
  }
  const mods=(tree&&tree.modules)||[];
  for(const mod of mods){
    const g=mod.group!=null?M.groups.find(x=>x.name===mod.group):null;
    const k0=topKeys.length;build(mod,[],inMask(g)?mask:modelRoot,0);
    if(g&&g.second&&topKeys.length>k0){                                                     // --second starts hidden
      const r=nodes.get(topKeys[k0]);if(!vis.has(topKeys[k0])&&!r.leaves.some(l=>vis.has(l)))for(const l of r.leaves)vis.set(l,false);
    }
  }
  const rest=[...meshByNode.keys()].filter(n=>!used.has(n));
  if(rest.length)build({name:'Other',children:rest.map(n=>({name:n.slice(n.indexOf(':')+1),node:n.slice(n.indexOf(':')+1),file:+n.slice(0,n.indexOf(':')),component:n,path:['Other',n]}))},[],modelRoot,0);
  // visibility lives on leaves only: an older branch-level flag (from a reload) becomes its leaves' flags
  for(const [k,v] of [...vis]){const r=nodes.get(k);if(r&&!r.leaf){vis.delete(k);if(v===false)for(const l of r.leaves)vis.set(l,false);}}
  modelRoot.updateMatrixWorld(true);
  for(const k of topKeys){
    const g=nodes.get(k).obj,b=new THREE.Box3().setFromObject(g);
    const c=b.isEmpty()?new THREE.Vector3():b.getCenter(new THREE.Vector3()).applyMatrix4(g.parent.matrixWorld.clone().invert());
    modules.push({key:k,group:g,centroid:c});
  }
  explodeParts.length=0;
  if(headMode)for(const m of meshList){
    if(!inMask(groupOf(m.userData.file)))continue;
    m.geometry.computeBoundingBox();m.updateMatrix();
    const centroid=m.geometry.boundingBox.getCenter(new THREE.Vector3()).applyMatrix4(m.matrix);
    explodeParts.push({mesh:m,position:m.position.clone(),centroid});
  }
  buildTreeDom();applyVisibility();applyExplode();
}

function buildTreeDom(){
  const root=$('tree');const st=root.scrollTop;root.innerHTML='';
  function row(k,rec,parentEl,isMod){
    const d=document.createElement('div');d.className='node'+(isMod?' mod':'');
    const line=document.createElement('div');line.className='line';
    const kids=rec.leaf?null:rec.obj.children;
    if(kids&&kids.length){
      const open=openState.get(k)??false;if(!open)d.classList.add('closed');
      const c=document.createElement('button');c.className='caret';c.textContent=open?'▾':'▸';
      c.onclick=()=>{const o=d.classList.toggle('closed');openState.set(k,!o);c.textContent=o?'▸':'▾';};
      line.append(c);
    }else{const s=document.createElement('span');s.className='spacer';line.append(s);}
    const cb=document.createElement('input');cb.type='checkbox';
    cb.onchange=()=>{for(const l of rec.leaves)viewHidden.delete(l);setVis(k,cb.checked);commit();};
    const nm=document.createElement('span');nm.className='nm';nm.textContent=rec.leaf?rec.leaf.name:rec.obj.name;nm.title=keyOf(rec.path);
    nm.onclick=()=>{select(k,false);commit();};nm.ondblclick=()=>{select(k,false);focusKey(k);};
    line.append(cb,nm);
    rec.cnt=null;rec.soloEl=null;
    if(!rec.leaf){const n=document.createElement('span');n.className='cnt';n.textContent=rec.count;line.append(n);rec.cnt=n;}
    if(isMod){const s=document.createElement('button');s.className='solo';s.textContent='solo';s.title='show only this module';s.onclick=()=>{solo(k);commit();};line.append(s);rec.soloEl=s;}
    d.append(line);rec.el=line;rec.cb=cb;parentEl.append(d);
    if(kids&&kids.length){
      const kd=document.createElement('div');kd.className='kids';d.append(kd);
      for(const ch of kids){const ck=ch.userData.key;const cr=nodes.get(ck);if(cr)row(ck,cr,kd,false);}
    }
  }
  for(const k of topKeys)row(k,nodes.get(k),root,true);
  root.scrollTop=st;
  renderTreeState();
}
// One source of truth for visibility: `vis` holds one flag per leaf (part). Every path that shows or hides parts
// (tree checkboxes, solo, show all, hide fasteners, view buttons, ?view= and the other URL params, undo/redo, reload)
// goes through setVis/applyVisibility, and the tree is rendered from `vis`: a branch is checked when all its parts
// show, unchecked when none do, indeterminate in between, and its count reads shown/total when partial.
const leafShown=l=>vis.get(l)??true;
function setVis(k,v,apply=true){const r=nodes.get(k);for(const l of (r?r.leaves:[k]))vis.set(l,v);if(apply)applyVisibility();}
function renderTreeState(){
  const shownMods=[];
  for(const [k,r] of nodes){
    let n=0;for(const l of r.leaves)if(leafShown(l))n++;
    const all=n===r.leaves.length;r.shown=n;
    if(r.cb){r.cb.checked=all;r.cb.indeterminate=!all&&n>0;}
    if(r.cnt){r.cnt.textContent=all?String(r.count):n+'/'+r.count;r.cnt.title=n+' of '+r.count+' parts shown';}
    if(r.el)r.el.classList.toggle('off',n===0);
  }
  for(const m of modules){const r=nodes.get(m.key);if(r&&r.shown>0)shownMods.push(m.key);}
  const soloKey=modules.length>1&&shownMods.length===1?shownMods[0]:null;
  for(const m of modules){const r=nodes.get(m.key);if(r&&r.soloEl)r.soloEl.classList.toggle('on',m.key===soloKey);}
}
function applyVisibility(){
  for(const [k,r] of nodes)r.obj.visible=r.leaf?leafShown(k):true;
  renderTreeState();
  if(sel&&!sel.rec.leaves.some(leafShown))clearSel();
  camLabel();dirty=true;
}
function effVisible(o){for(;o;o=o.parent)if(!o.visible)return false;return true;}
function solo(k){for(const m of modules)setVis(m.key,m.key===k,false);applyVisibility();}
function hideFasteners(){for(const [k,r] of nodes)if(!r.leaf&&r.obj.name==='Fasteners')setVis(k,false,false);applyVisibility();}
$('showall').onclick=()=>{for(const [k,r] of nodes)if(r.leaf)vis.set(k,true);viewHidden.clear();applyVisibility();commit();};
$('hidefast').onclick=()=>{hideFasteners();commit();};
$('collapse').onclick=()=>{openState.clear();buildTreeDom();};

// ---- explode: fitted pieces move away from the fixed scan; other assemblies explode by module ----
let radius=300;const explodeO=new THREE.Vector3();
function resetExplode(){
  for(const p of explodeParts)p.mesh.position.copy(p.position);
  for(const m of modules)m.group.position.set(0,0,0);
}
function applyExplode(){
  const e=+$('explode').value;$('explodev').textContent=e.toFixed(2);
  resetExplode();
  if(headMode){
    modelRoot.updateMatrixWorld(true);
    const headBox=new THREE.Box3();
    for(const m of meshList)if(!inMask(groupOf(m.userData.file)))headBox.expandByObject(m);
    const origin=headBox.isEmpty()?explodeO:headBox.getCenter(new THREE.Vector3());
    for(const p of explodeParts){
      // Compute the outward direction in world space, then convert the displaced centre back to the
      // mesh's parent frame. Rotation, translation and nonuniform scale of the fitted mask still work.
      const c=p.centroid.clone().applyMatrix4(p.mesh.parent.matrixWorld),offset=c.clone().sub(origin);
      const d=offset.length()>1?offset.clone().normalize():new THREE.Vector3(0,0,1);
      c.addScaledVector(offset,1.5*e).addScaledVector(d,0.35*radius*e);
      const local=p.mesh.parent.worldToLocal(c).sub(p.centroid);
      if(e!==0)p.mesh.position.copy(p.position).add(local);
    }
  }else for(const m of modules){
    const c=m.centroid.clone().sub(explodeO),d=c.length()>1?c.clone().normalize():new THREE.Vector3(0,0,1);
    m.group.position.copy(c).multiplyScalar(1.5*e).addScaledVector(d,0.35*radius*e);
  }
  camLabel();dirty=true;
}
$('explode').addEventListener('input',applyExplode);
$('explode').addEventListener('change',()=>commit());

// ---- section planes (local clipping on every material) ----
let bbox=new THREE.Box3(new THREE.Vector3(-150,-150,-125),new THREE.Vector3(150,150,200));
function setRanges(){
  for(const [id,a] of [['clipZ','z'],['clipX','x']]){
    const s=$(id),lo=Math.floor(bbox.min[a])-1,hi=Math.ceil(bbox.max[a])+1,keep=s.dataset.init?+s.value:null;
    const def=a==='x'&&M.explode_origin==='origin'?0:Math.round((lo+hi)/2);
    s.min=lo;s.max=hi;s.value=keep??def;s.dataset.init=1;
  }
  applyClip();
}
function applyClip(){
  clipPlanes.length=0;
  for(const [id,p,n] of [['clipZ',planeZ,new THREE.Vector3(0,0,-1)],['clipX',planeX,new THREE.Vector3(-1,0,0)]]){
    const v=+$(id).value,flip=$(id+'flip').checked;$(id+'v').textContent=v+' mm';
    if(flip)p.set(n.clone().negate(),-v);else p.set(n,v);
    if($(id+'on').checked)clipPlanes.push(p);
  }
  for(const m of meshList)(Array.isArray(m.material)?m.material:[m.material]).forEach(x=>x.needsUpdate=true);
  camLabel();dirty=true;
}
const CLIP_IDS=['clipZ','clipX','clipZon','clipXon','clipZflip','clipXflip'];
for(const id of CLIP_IDS){$(id).addEventListener('input',applyClip);$(id).addEventListener('change',()=>commit());}   // commit on release

// ---- picking: hover tooltip, click select, double-click focus ----
const ray=new THREE.Raycaster(),ndc=new THREE.Vector2();
function pick(ev){
  const r=canvas.getBoundingClientRect();
  let list=meshList.filter(effVisible);
  if(fishOn()){
    const d=fishDir(ev);if(!d)return null;ray.set(new THREE.Vector3(...camMode.eye),d);
    const obs=new Set();for(const o of observerObjs())o.traverse(m=>obs.add(m));list=list.filter(m=>!obs.has(m));
  }else{ndc.set(((ev.clientX-r.left)/r.width)*2-1,-((ev.clientY-r.top)/r.height)*2+1);ray.setFromCamera(ndc,cam);}
  const hits=ray.intersectObjects(list,false);
  for(const h of hits)if(clipPlanes.every(p=>p.distanceToPoint(h.point)>=0))return h.object;
  return null;
}
let sel=null;
function paint(objs,on){
  for(const o of objs)o.traverse(m=>{if(!m.isMesh)return;(Array.isArray(m.material)?m.material:[m.material]).forEach(x=>{
    if(!x.emissive)return;if(on){x.emissive.copy(HL);x.emissiveIntensity=0.55;}else{x.emissive.copy(x.userData.emissive0||new THREE.Color(0));x.emissiveIntensity=1;}
  });});
  dirty=true;
}
const HINT='Click a part to select it. Double-click to focus. Esc clears. Cmd/Ctrl+Z undoes.';
function clearSel(){if(sel){paint(sel.objs,false);sel.rec.el&&sel.rec.el.classList.remove('sel');}sel=null;
  $('selname').textContent=HINT;$('selpath').textContent='';}
function select(k,reveal=true){
  const rec=nodes.get(k);if(!rec)return;clearSel();
  sel={key:k,rec,objs:[rec.obj]};paint(sel.objs,true);
  $('selname').textContent=rec.leaf?rec.leaf.name:rec.obj.name+' ('+rec.count+' parts)';
  $('selpath').textContent=keyOf(rec.path)+(rec.leaf&&rec.leaf.component&&rec.leaf.component!==rec.leaf.name?'   ['+rec.leaf.component+(rec.leaf.kind?', '+rec.leaf.kind:'')+']':'');
  if(reveal){
    for(let i=1;i<rec.path.length;i++){const pk=keyOf(rec.path.slice(0,i));openState.set(pk,true);
      const pr=nodes.get(pk);if(pr&&pr.el){const d=pr.el.parentElement;d.classList.remove('closed');const c=pr.el.querySelector('.caret');if(c)c.textContent='▾';}}
  }
  if(rec.el){rec.el.classList.add('sel');if(reveal)rec.el.scrollIntoView({block:'nearest'});}
}
let tween=null;
function focusBox(b,animate=true){
  const c=b.getCenter(new THREE.Vector3()),r=Math.max(2,b.getBoundingSphere(new THREE.Sphere()).radius)*1.25;
  const dir=cam.position.clone().sub(controls.target).normalize();
  const to={t:c,p:cam.isOrthographicCamera?c.clone().addScaledVector(dir,Math.max(2000,r*6)):c.clone().addScaledVector(dir,r/Math.tan(cam.fov*Math.PI/360)),
    z:cam.isOrthographicCamera?OB/r:cam.zoom};
  const from={t:controls.target.clone(),p:cam.position.clone(),z:cam.zoom};
  if(!animate){controls.target.copy(to.t);cam.position.copy(to.p);cam.zoom=to.z;cam.updateProjectionMatrix();controls.update();return;}
  tween={from,to,t0:performance.now()};
}
function focusKey(k){if(fishOn())return;const r=nodes.get(k);if(!r)return;const b=new THREE.Box3().setFromObject(r.obj);if(!b.isEmpty())focusBox(b);}
let down=null,hoverT=0,lastMove=null;
canvas.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];$('tip').style.display='none';});
canvas.addEventListener('wheel',()=>{$('tip').style.display='none';},{passive:true});
canvas.addEventListener('pointerup',e=>{
  if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4||(tc&&tc.dragging))return;down=null;
  const m=pick(e);if(m)select(m.userData.key);else clearSel();commit();
});
canvas.addEventListener('dblclick',e=>{const m=pick(e);if(m){select(m.userData.key);if(!fishOn())focusKey(m.userData.key);commit();}});
canvas.addEventListener('pointermove',e=>{lastMove=e;});
canvas.addEventListener('pointerleave',()=>{lastMove=null;$('tip').style.display='none';});
function hover(now){
  if(!lastMove||now-hoverT<70||(down&&lastMove.buttons))return;hoverT=now;
  const e=lastMove;lastMove=null;const m=pick(e),tip=$('tip');
  if(!m){tip.style.display='none';canvas.style.cursor='';return;}
  const r=host.getBoundingClientRect();tip.textContent=m.userData.leaf?m.userData.leaf.name:m.name;
  tip.style.left=(e.clientX-r.left+14)+'px';tip.style.top=(e.clientY-r.top+14)+'px';tip.style.display='block';canvas.style.cursor='pointer';
}

// ---- undo / redo: one history of committed state (visibility, section, explode, selection, camera buttons,
// projection, movable group pose). Orbit drags are not in it; a camera snapshot is only restored when the step
// being undone or redone was itself a camera action. ----
const hist=[];let hp=-1,ready=false,restoring=false;
const HMAX=200;
function snapCam(){return {proj:cam===ortho?'o':'p',p:cam.position.toArray(),t:controls.target.toArray(),z:cam.zoom,v:activeView,
  cv:camMode?camMode.name:null,pb:projBefore===ortho?'o':projBefore===persp?'p':null};}
function snapshot(){
  return {vis:[...vis].sort(),vh:[...viewHidden].sort(),
    clip:CLIP_IDS.map(id=>{const e=$(id);return e.type==='checkbox'?e.checked:+e.value;}),
    ex:+$('explode').value,grid:showGrid,sel:sel?sel.key:null,cam:snapCam(),
    mask:mask?{p:mask.position.toArray(),q:mask.quaternion.toArray(),s:mask.scale.toArray()}:null};
}
const PARTS=['vis','clip','ex','grid','sel','cam','mask'];
const J=JSON.stringify,partOf=(s,k)=>k==='vis'?J([s.vis,s.vh]):J(s[k]);
function pushHist(){
  const n=snapshot();
  if(hp>=0&&PARTS.every(k=>partOf(hist[hp],k)===partOf(n,k)))return false;
  hist.length=hp+1;hist.push(n);if(hist.length>HMAX)hist.shift();hp=hist.length-1;updUndo();return true;
}
// a committed change: the current entry first takes the live camera (orbits since then are not steps), then the
// new state is pushed. Camera actions call camAction, which records the camera before it moves.
function commit(){
  if(!ready||restoring)return;
  if(hp>=0)hist[hp].cam=snapCam();
  if(pushHist()&&mask&&partOf(hist[hp],'mask')!==(hp>0?partOf(hist[hp-1],'mask'):null))poseChanged();
}
function camAction(fn){
  if(ready&&!restoring&&hp>=0)hist[hp].cam=snapCam();
  fn();
  if(!ready||restoring)return;
  pushHist();
}
function restoreCam(c){
  tween=null;const cv=c.cv&&VIEWS.find(v=>v.name===c.cv);
  camMode=cv||null;persp.fov=FOV;persp.updateProjectionMatrix();
  projBefore=cv?(c.pb==='o'?ortho:persp):null;camUI();
  const C=c.proj==='o'?ortho:persp;useCam(C);
  C.position.fromArray(c.p);controls.target.fromArray(c.t);C.zoom=c.z;C.up.set(0,0,1);C.updateProjectionMatrix();C.lookAt(controls.target);controls.update();
  markView(c.v);
}
function applyHist(i){
  if(i<0||i>=hist.length||i===hp)return;
  const cur=hist[hp],t=hist[i];restoring=true;
  try{
    if(partOf(cur,'vis')!==partOf(t,'vis')){vis.clear();for(const [k,v] of t.vis)vis.set(k,v);viewHidden.clear();for(const k of t.vh)viewHidden.add(k);applyVisibility();}
    if(partOf(cur,'clip')!==partOf(t,'clip')){CLIP_IDS.forEach((id,j)=>{const e=$(id);if(e.type==='checkbox')e.checked=t.clip[j];else e.value=t.clip[j];});applyClip();}
    if(cur.ex!==t.ex){$('explode').value=t.ex;applyExplode();}
    if(cur.grid!==t.grid)setGridVisible(t.grid);
    if(partOf(cur,'cam')!==partOf(t,'cam'))restoreCam(t.cam);
    if(mask&&partOf(cur,'mask')!==partOf(t,'mask')){mask.position.fromArray(t.mask.p);mask.quaternion.fromArray(t.mask.q);mask.scale.fromArray(t.mask.s);showPose();poseChanged();}
    if(cur.sel!==t.sel||(t.sel&&!sel)){if(t.sel&&nodes.has(t.sel))select(t.sel);else clearSel();}
  }finally{restoring=false;}
  hp=i;updUndo();dirty=true;
}
const undo=()=>applyHist(hp-1),redo=()=>applyHist(hp+1);
function updUndo(){$('undo').disabled=hp<=0;$('redo').disabled=hp>=hist.length-1;
  $('undo').title='Undo (Cmd/Ctrl+Z), '+Math.max(0,hp)+' step'+(hp===1?'':'s');$('redo').title='Redo (Shift+Cmd/Ctrl+Z or Ctrl+Y), '+(hist.length-1-hp)+' step'+(hist.length-1-hp===1?'':'s');}
$('undo').onclick=undo;$('redo').onclick=redo;
const typing=t=>!!t&&((t.tagName==='INPUT'&&t.type!=='checkbox'&&t.type!=='range'&&document.activeElement===t)||t.tagName==='TEXTAREA'||t.isContentEditable);
addEventListener('keydown',e=>{
  if(!(e.metaKey||e.ctrlKey)||e.altKey||typing(e.target))return;
  const k=e.key.toLowerCase();if(k==='z'){e.preventDefault();e.shiftKey?redo():undo();}
  else if(k==='y'&&e.ctrlKey){e.preventDefault();redo();}
});
addEventListener('keydown',e=>{
  if(e.metaKey||e.ctrlKey||e.altKey||typing(e.target))return;
  if(e.key==='Escape'){
    if(camMode){camAction(()=>{const v=VIEWS.find(x=>x.name===viewBefore&&x.kind!=='camera');if(v)applyView(v);else fitAll();});return;}
    if(sel){clearSel();commit();}return;
  }
  if((e.key==='f'||e.key==='F')&&sel)return focusKey(sel.key);
  if(mask)maskKey(e);
});

// ---- movable group ----
const fmt=(v,n)=>(+v.toFixed(n)).toString();
const F={px:['position','x',2],py:['position','y',2],pz:['position','z',2],rx:['rotation','x',2],ry:['rotation','y',2],rz:['rotation','z',2],sx:['scale','x',4],sy:['scale','y',4],sz:['scale','z',4]};
const lockOn=()=>$('lock').checked;
let lastScale=new THREE.Vector3(1,1,1);
function showPose(){
  if(!mask)return;
  for(const[id,[k,a,n]]of Object.entries(F)){
    const el=$(id);if(document.activeElement===el)continue;
    el.value=fmt(k==='rotation'?mask.rotation[a]*R2D:mask[k][a],n);
  }
  lastScale.copy(mask.scale);
  const dp=mask.position.clone().sub(loadedPose.p);
  const dq=loadedPose.q.clone().invert().multiply(mask.quaternion);
  const ang=2*Math.acos(Math.min(1,Math.abs(dq.w)))*R2D;
  const ds=mask.scale.clone().divide(loadedPose.s);
  $('delta').textContent=`moved ${[dp.x,dp.y,dp.z].map(v=>fmt(v,1)).join(', ')} mm (|${fmt(dp.length(),1)}|), turned ${fmt(ang,1)} deg, scale x${fmt(ds.x,3)}`;
  if(headMode&&explodeParts.length)applyExplode();
  dirty=true;
}
let saveTimer=null,saving=Promise.resolve();
function poseJSON(){
  mask.updateMatrix();
  const m=mask.matrix.elements.slice();
  const rows=[0,1,2,3].map(r=>[0,1,2,3].map(c=>m[c*4+r]));
  const head=M.files.find(f=>!inMask(groupOf(f.i)));
  return {overlay:M.movable,pieces:maskPieces(),head:headMode&&head?head.name:null,matrix:m,matrix_rows:rows,
    position_mm:mask.position.toArray(),rotation_deg:[mask.rotation.x,mask.rotation.y,mask.rotation.z].map(v=>v*R2D),rotation_order:'XYZ',
    scale:mask.scale.toArray(),frame:'mm, Z up, model frame of the loaded files',saved_at:new Date().toISOString()};
}
const maskPieces=()=>{const g=M.groups.find(x=>x.movable);return g?g.files.map(i=>M.files[i].name):[];};
async function save(){
  clearTimeout(saveTimer);saveTimer=null;
  $('posestat').textContent='saving...';
  saving=saving.then(async()=>{
    try{
      const r=await fetch('/pose',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(poseJSON())});
      const j=await r.json();if(!r.ok)throw new Error(j.error||r.status);
      $('posestat').textContent='saved '+new Date().toLocaleTimeString([],{hour12:false});$('saved').textContent=j.saved;
    }catch(e){$('posestat').textContent='save failed: '+e.message;console.warn(e);}
  });
  return saving;
}
function poseChanged(){clearTimeout(saveTimer);saveTimer=setTimeout(save,500);}
function setMode(m){tc.setMode(m);[['mT','translate'],['mR','rotate'],['mS','scale']].forEach(([id,mm])=>$(id).classList.toggle('on',mm===m));dirty=true;}
function toggleSpace(){tc.setSpace(tc.space==='local'?'world':'local');$('mSp').textContent=tc.space;dirty=true;}
function maskKey(e){
  const step=e.shiftKey?5:1,k=e.key,kl=k.toLowerCase();
  if(k==='Shift'){tc.setTranslationSnap(5);tc.setRotationSnap(5*D2R);tc.setScaleSnap(0.01);return;}
  if(kl==='w')return setMode('translate');if(kl==='e')return setMode('rotate');if(kl==='r')return setMode('scale');
  if(kl==='x')return toggleSpace();
  let moved=true;cam.updateMatrixWorld();
  const right=new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld,0),up=new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld,1);
  if(k==='ArrowLeft')mask.position.addScaledVector(right,-step);
  else if(k==='ArrowRight')mask.position.addScaledVector(right,step);
  else if(k==='ArrowUp')mask.position.addScaledVector(up,step);
  else if(k==='ArrowDown')mask.position.addScaledVector(up,-step);
  else if(k==='PageUp')mask.position.z+=step;
  else if(k==='PageDown')mask.position.z-=step;
  else if(k==='['||k==='{')mask.rotation.x-=D2R;
  else if(k===']'||k==='}')mask.rotation.x+=D2R;
  else moved=false;
  if(moved){e.preventDefault();showPose();commit();}
}
addEventListener('keyup',e=>{if(e.key==='Shift'&&tc){tc.setTranslationSnap(null);tc.setRotationSnap(null);tc.setScaleSnap(null);}});
function setupMask(){
  mask=new THREE.Group();mask.name=M.movable;
  tc=new TransformControls(cam,canvas);
  tc.addEventListener('dragging-changed',e=>{controls.enabled=!e.value;if(!e.value){showPose();commit();}});
  tc.addEventListener('objectChange',()=>{if(lockOn()&&tc.getMode()==='scale'){const s=mask.scale,d=[s.x-lastScale.x,s.y-lastScale.y,s.z-lastScale.z].map(Math.abs);mask.scale.setScalar([s.x,s.y,s.z][d.indexOf(Math.max(...d))]);}showPose();});
  tc.addEventListener('change',()=>{dirty=true;});
  scene.add(tc);
  for(const[id,[k,a]]of Object.entries(F)){
    $(id).addEventListener('change',e=>{
      const v=parseFloat(e.target.value);if(!isFinite(v))return showPose();
      if(k==='rotation')mask.rotation[a]=v*D2R;else if(k==='scale'&&lockOn())mask.scale.setScalar(v);else mask[k][a]=v;
      e.target.blur();showPose();commit();
    });
  }
  $('lock').onchange=()=>{if(lockOn()){mask.scale.setScalar(mask.scale.x);showPose();commit();}};
  $('mT').onclick=()=>setMode('translate');$('mR').onclick=()=>setMode('rotate');$('mS').onclick=()=>setMode('scale');$('mSp').onclick=toggleSpace;
  $('reset').onclick=()=>{mask.position.copy(loadedPose.p);mask.quaternion.copy(loadedPose.q);mask.scale.copy(loadedPose.s);showPose();commit();};
  $('resume').onclick=()=>{applySaved();commit();};
  $('save').onclick=save;
  $('op').oninput=e=>{const o=+e.target.value;$('opv').textContent=o.toFixed(2);
    for(const m of meshList){if(!inMask(groupOf(m.userData.file)))continue;const x=m.material;x.opacity=o;x.transparent=o<1;x.depthWrite=o>=1;x.needsUpdate=true;}dirty=true;};
  $('mname').textContent=M.movable;$('maskbox').hidden=false;
  fetch('/pose-info').then(r=>r.json()).then(j=>{if(!$('saved').textContent)$('saved').textContent='saves to '+j.save_to;}).catch(()=>{});
}
function applySaved(){if(!savedPose)return;savedPose.decompose(mask.position,mask.quaternion,mask.scale);mask.rotation.setFromQuaternion(mask.quaternion);showPose();}
async function initMask(){
  tc.attach(mask);
  const n=maskPieces().length;$('mcount').textContent='('+n+' piece'+(n===1?'':'s')+')';
  $('opv').textContent=(+$('op').value).toFixed(2);
  try{
    const sp=await (await fetch('/pose')).json(),names=maskPieces();
    if(Array.isArray(sp.matrix)&&sp.matrix.length===16&&sp.matrix.every(Number.isFinite)&&Array.isArray(sp.pieces)&&sp.pieces.length===names.length&&sp.pieces.every((x,i)=>x===names[i])){
      savedPose=new THREE.Matrix4().fromArray(sp.matrix);$('resume').disabled=false;
      $('saved').textContent='resumed pose saved '+(sp.saved_at||'(unknown time)');applySaved();
    }
  }catch(e){console.warn(e);}
  showPose();
}

// ---- loading and live reload ----
let loadedSig=null,loadedMtime=null,loadedTree=null,status=null,busy=false,retryTimer=null;
const hhmmss=t=>new Date(t*1000).toLocaleTimeString([],{hour12:false});
function showStatus(flash){
  const s=status,el=$('status');if(!s)return;
  const pending=s.sheet_mtime!=null&&s.mtime!=null&&s.sheet_mtime<s.mtime-1;
  let txt=(s.build_ok==null&&s.sheet_mtime==null?'updated ':'rebuilt ')+hhmmss(loadedMtime??s.mtime);
  let cls='';
  if(pending){txt+=', checks pending';cls='pending';}
  else if(s.build_ok===true){txt+=', build ok';cls='ok';}
  else if(s.build_ok===false){txt+=', build FAILED';cls='bad';}
  if(!pending&&s.interference_pairs!=null)txt+=', '+s.interference_pairs+' interference'+(s.interference_pairs===1?'':'s');
  if(!pending&&s.design_violations!=null)txt+=', '+s.design_violations+' design violation'+(s.design_violations===1?'':'s');
  el.textContent=txt;el.className=cls;el.title=(s.files||[]).map(f=>f.name).join(', ')||s.glb||'';
  if(flash){el.classList.add('flash');setTimeout(()=>el.classList.remove('flash'),900);}
}
async function fetchTree(){tree=await (await fetch('/tree.json?ts='+Date.now())).json();}
async function loadFile(f,fs){
  const r=await fetch(f.url+'?v='+fs.mtime);if(!r.ok)throw new Error(f.name+' '+r.status);
  const buf=await r.arrayBuffer();if(fs.size&&buf.byteLength!==fs.size)throw new Error('size changed');
  const out=[];
  if(f.ext==='.glb'){
    const gltf=await gltfLoader.parseAsync(buf,'');const json=gltf.parser.json;
    gltf.scene.traverse(o=>{
      if(!o.isMesh)return;const a=gltf.parser.associations.get(o);
      let nm=(a&&a.nodes!==undefined&&json.nodes[a.nodes]&&json.nodes[a.nodes].name)||o.userData.name||o.name;
      if(!a||a.nodes===undefined){const pa=gltf.parser.associations.get(o.parent);if(pa&&pa.nodes!==undefined)nm=json.nodes[pa.nodes].name||nm;}
      o.updateWorldMatrix(true,false);
      o.matrix.copy(o.matrixWorld);o.matrix.decompose(o.position,o.quaternion,o.scale);
      prepMesh(o,true);out.push([nm,o]);
    });
  }else{
    const geo=(f.ext==='.ply'?plyLoader:stlLoader).parse(buf);
    if(!geo.hasAttribute('normal')||f.ext==='.stl')geo.computeVertexNormals();
    const m=new THREE.Mesh(geo,meshMaterial(f,geo));m.name=f.name;prepMesh(m,false);out.push([f.name,m]);
  }
  for(const [,m] of out)m.userData.file=f.i;
  return out;
}
async function loadModel(s){
  // a file may still be mid-write: incomplete GLB header length or a short read -> retry in 1 s
  if(!s.complete)throw new Error('file incomplete');
  M=await (await fetch('/models.json?ts='+Date.now())).json();
  const next=new Map(),list=[];
  for(const f of M.files){
    const fs=(s.files||[]).find(x=>x.i===f.i)||{};
    for(const [nm,m] of await loadFile(f,fs)){let k=f.i+':'+nm;for(let j=2;next.has(k);j++)k=f.i+':'+nm+' #'+j;next.set(k,m);list.push(m);}
  }
  await fetchTree();
  const old=meshList;const selKey=sel&&sel.key;clearSel();
  meshByNode=next;meshList=list;
  for(const m of modules)m.group.position.set(0,0,0);
  assemble();
  for(const m of old){m.geometry.dispose();(Array.isArray(m.material)?m.material:[m.material]).forEach(x=>x.dispose());}
  const b=new THREE.Box3();resetExplode();
  const mp=mask?[mask.position.clone(),mask.quaternion.clone(),mask.scale.clone()]:null;
  if(mask){mask.position.set(0,0,0);mask.quaternion.identity();mask.scale.set(1,1,1);}
  modelRoot.updateMatrixWorld(true);for(const m of list)b.expandByObject(m);
  if(mp){mask.position.copy(mp[0]);mask.quaternion.copy(mp[1]);mask.scale.copy(mp[2]);modelRoot.updateMatrixWorld(true);}
  if(!b.isEmpty()){bbox=b;radius=b.getBoundingSphere(new THREE.Sphere()).radius;}
  if(M.explode_origin!=='origin')b.getCenter(explodeO);else explodeO.set(0,0,0);
  const ext=Math.max(Math.abs(bbox.min.x),Math.abs(bbox.max.x),Math.abs(bbox.min.y),Math.abs(bbox.max.y));
  setGrid(Math.max(600,Math.ceil(2.4*ext/100)*100));
  applyExplode();
  setRanges();
  if(fitted){VIEWS=fittedViews();renderViewButtons();}
  if(selKey&&nodes.has(selKey))select(selKey,false);
  const first=loadedSig===null;loadedSig=s.sig;loadedMtime=s.mtime;loadedTree=s.tree_mtime;
  if(first){
    $('title').textContent=M.title||M.name;document.title=(M.title||M.name)+' viewer';
    const v0=VIEWS.find(v=>v.name===(headMode?'front':'iso'));if(v0)applyView(v0);else fitAll();
    applyParams();
    if(mask)await initMask();
    ready=true;pushHist();
  }
  showStatus(!first);
  let tris=0;for(const m of list)tris+=m.geometry.index?m.geometry.index.count/3:m.geometry.attributes.position.count/3;
  log('info',(first?'ready':'reloaded')+' '+list.length+' meshes, '+Math.round(tris)+' triangles, '+modules.length+' modules, sig '+s.sig.slice(0,200));
}
// URL parameters for links and scripted checks: ?view=hub (?view=camera: the UC-261 fisheye lens view)&proj=persp&explode=0.5&clipz=60&clipx=0&hide=fasteners
// &solo=Rotor&select=<display name or component name>
function applyParams(){
  const q=new URLSearchParams(location.search),num=k=>q.has(k)&&isFinite(+q.get(k))?+q.get(k):null;
  if(q.get('proj')==='persp')setCam(persp);
  const v=VIEWS.find(x=>x.name===q.get('view'));if(v)applyView(v);
  if(num('explode')!==null){$('explode').value=num('explode');applyExplode();}
  if(q.has('grid'))setGridVisible(q.get('grid')!=='0');
  for(const [k,id] of [['clipz','clipZ'],['clipx','clipX']])if(num(k)!==null){$(id).value=num(k);$(id+'on').checked=true;}
  applyClip();
  if(q.get('hide')==='fasteners')hideFasteners();
  if(q.get('solo')){const m=modules.find(x=>x.group.name===q.get('solo'));if(m)solo(m.key);}
  if(q.get('select')){const w=q.get('select');for(const [k,r] of nodes)if(r.leaf&&(r.leaf.name===w||r.leaf.component===w)){select(k);break;}}
}
// a small handle for scripted checks (CDP) and the console
window.viewer={get camera(){return cam;},controls,nodes,vis,modules,select,clearSel,undo,redo,commit,
  applyView:n=>camAction(()=>applyView(VIEWS.find(v=>v.name===n))),
  // camera view: the parts a lens ray at client (x, y) passes, nearest first, transparent ones marked
  lensHits:(x,y)=>{if(!fishOn())return null;const d=fishDir({clientX:x,clientY:y});if(!d)return [];ray.set(new THREE.Vector3(...camMode.eye),d);
    const obs=new Set();for(const o of observerObjs())o.traverse(m=>obs.add(m));
    return ray.intersectObjects(meshList.filter(m=>effVisible(m)&&!obs.has(m)),false).slice(0,6).map(h=>(h.object.userData.leaf?h.object.userData.leaf.component:h.object.name)+(h.object.material.transparent?' (clear)':'')+' @'+h.distance.toFixed(1));},
  state:()=>({ready,loadedMtime,soft:SOFT,meshes:meshList.length,cam:cam.position.toArray(),target:controls.target.toArray(),zoom:cam.zoom,
    hidden:[...vis].filter(([,v])=>!v).map(([k])=>k),clip:clipPlanes.map(p=>[p.normal.toArray(),p.constant]),
    explode:+$('explode').value,grid:showGrid,view:activeView,cameraView:camMode?camMode.name:null,fov:cam.isPerspectiveCamera?cam.fov:null,
    proj:cam===ortho?'ortho':'persp',projDisabled:$('proj').disabled,fisheye:fishOn(),fishRect:FISH.rect,camLabel:$('camnote').hidden?null:$('camnote-t').textContent,orbit:controls.enabled,selected:sel&&sel.key,renders,ctrlTrue,status:$('status').textContent,
    history:hist.length,hp,undo:!$('undo').disabled,redo:!$('redo').disabled,
    mask:mask?{p:mask.position.toArray(),pieces:maskPieces()}:null})};
async function poll(){
  if(busy)return;busy=true;clearTimeout(retryTimer);
  try{
    status=await (await fetch('/status.json?ts='+Date.now())).json();
    if(status.sig!==loadedSig){
      try{await loadModel(status);}
      catch(e){if(loadedSig===null)$('status').textContent='loading... ('+e.message+', retrying)';console.warn('load',e.message);retryTimer=setTimeout(poll,1000);}
    }else if(status.tree_mtime!==loadedTree){
      loadedTree=status.tree_mtime;await fetchTree();const k=sel&&sel.key;assemble();if(k&&nodes.has(k))select(k,false);showStatus(true);
    }else showStatus(false);
  }catch(e){$('status').textContent='server not reachable';$('status').className='bad';}
  finally{busy=false;}
}
(async()=>{
  try{M=await (await fetch('/models.json')).json();}catch(e){$('status').textContent='server not reachable';return;}
  $('title').textContent=M.title||M.name;
  if(M.movable){
    setupMask();headMode=M.files.some(f=>!inMask(groupOf(f.i)));
    // Coloured prints start opaque; the opacity slider still exposes the scan for fit inspection.
    const colored=M.files.some(f=>inMask(groupOf(f.i))&&f.color);
    $('op').value=headMode&&!colored?0.45:1;
  }
  await loadViews();await poll();setInterval(poll,2000);
})();

// ---- software preview (no WebGL) ----
const lin2s=c=>c<=0.0031308?c*12.92:1.055*Math.pow(c,1/2.4)-0.055;
let sv=new Float32Array(0),sw=new Float32Array(0),zb=new Float32Array(0);const cd=new Float64Array(6);
function softRender(){
  const W=host.clientWidth,H=host.clientHeight;if(!W||!H)return;
  if(canvas.width!==W||canvas.height!==H){canvas.width=W;canvas.height=H;}
  const ctx=canvas.getContext('2d');ctx.fillStyle='#e5e7eb';ctx.fillRect(0,0,W,H);
  scene.updateMatrixWorld();cam.updateMatrixWorld();
  const VP=new THREE.Matrix4().multiplyMatrices(cam.projectionMatrix,cam.matrixWorldInverse);
  const toS=(v)=>{const p=v.clone().applyMatrix4(VP);return [(p.x*0.5+0.5)*W,(0.5-p.y*0.5)*H,p.z];};
  // grid at Z 0 and the axes, behind the model
  ctx.lineWidth=1;const half=GRID.size/2,nc=Math.round(half/GRID.cell);
  for(let i=-nc;showGrid&&i<=nc;i++){
    ctx.strokeStyle=i===0?'#64748b':'#abb2bd';
    for(const [a,b] of [[[i*GRID.cell,-half,0],[i*GRID.cell,half,0]],[[-half,i*GRID.cell,0],[half,i*GRID.cell,0]]]){
      const A=toS(new THREE.Vector3(...a)),B=toS(new THREE.Vector3(...b));
      if(Math.abs(A[2])>1||Math.abs(B[2])>1)continue;ctx.beginPath();ctx.moveTo(A[0],A[1]);ctx.lineTo(B[0],B[1]);ctx.stroke();
    }
  }
  const img=ctx.getImageData(0,0,W,H),px=new Uint32Array(img.data.buffer);
  if(zb.length!==W*H)zb=new Float32Array(W*H);zb.fill(Infinity);
  const L=new THREE.Vector3(0.3,0.5,1).normalize().transformDirection(cam.matrixWorld);
  const camPos=cam.position,fwd=new THREE.Vector3(0,0,-1).transformDirection(cam.matrixWorld),ortho=cam.isOrthographicCamera;
  const Mx=new THREE.Matrix4(),order=meshList.filter(effVisible);
  order.sort((a,b)=>(a.material.transparent?1:0)-(b.material.transparent?1:0));
  for(const mesh of order){
    const g=mesh.geometry,pa=g.attributes.position,n=pa.count,mat=mesh.material,col=mat.vertexColors?g.attributes.color:null;
    if(sv.length<n*3){sv=new Float32Array(n*3);sw=new Float32Array(n*3);}
    Mx.multiplyMatrices(VP,mesh.matrixWorld);const m=Mx.elements,e=mesh.matrixWorld.elements;
    for(let i=0;i<n;i++){
      const x=pa.getX(i),y=pa.getY(i),z=pa.getZ(i);
      sw[i*3]=e[0]*x+e[4]*y+e[8]*z+e[12];sw[i*3+1]=e[1]*x+e[5]*y+e[9]*z+e[13];sw[i*3+2]=e[2]*x+e[6]*y+e[10]*z+e[14];
      const w=m[3]*x+m[7]*y+m[11]*z+m[15];
      if(w<=1e-6){sv[i*3+2]=NaN;continue;}
      sv[i*3]=((m[0]*x+m[4]*y+m[8]*z+m[12])/w*0.5+0.5)*W;sv[i*3+1]=(0.5-(m[1]*x+m[5]*y+m[9]*z+m[13])/w*0.5)*H;sv[i*3+2]=(m[2]*x+m[6]*y+m[10]*z+m[14])/w;
    }
    const em=mat.emissive&&mat.emissive.equals(HL)?0.35:0,op=mat.transparent?mat.opacity:1;
    const br=mat.color.r,bg=mat.color.g,bb=mat.color.b;
    const idx=g.index?g.index.array:null,tcn=idx?idx.length/3:n/3;
    for(let t=0;t<tcn;t++){
      const i0=idx?idx[t*3]:t*3,i1=idx?idx[t*3+1]:t*3+1,i2=idx?idx[t*3+2]:t*3+2;
      const z0=sv[i0*3+2],z1=sv[i1*3+2],z2=sv[i2*3+2];
      if(!(z0>=-1&&z0<=1&&z1>=-1&&z1<=1&&z2>=-1&&z2<=1))continue;
      const ax=sw[i0*3],ay=sw[i0*3+1],az=sw[i0*3+2],ux=sw[i1*3]-ax,uy=sw[i1*3+1]-ay,uz=sw[i1*3+2]-az,vx=sw[i2*3]-ax,vy=sw[i2*3+1]-ay,vz=sw[i2*3+2]-az;
      // clip planes: drop triangles fully outside, test per pixel (distances interpolated) where one crosses
      let cut=0,out=false;
      for(let q=0;q<clipPlanes.length;q++){
        const p=clipPlanes[q],nx_=p.normal.x,ny_=p.normal.y,nz_=p.normal.z,c_=p.constant;
        const d0=nx_*ax+ny_*ay+nz_*az+c_,d1=d0+nx_*ux+ny_*uy+nz_*uz,d2=d0+nx_*vx+ny_*vy+nz_*vz;
        if(d0<0&&d1<0&&d2<0){out=true;break;}
        if(d0<0||d1<0||d2<0){cd[cut*3]=d0;cd[cut*3+1]=d1;cd[cut*3+2]=d2;cut++;}
      }
      if(out)continue;
      let nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;const nl=Math.hypot(nx,ny,nz)||1;nx/=nl;ny/=nl;nz/=nl;
      const dx=ortho?-fwd.x:camPos.x-ax,dy=ortho?-fwd.y:camPos.y-ay,dz=ortho?-fwd.z:camPos.z-az;
      if(nx*dx+ny*dy+nz*dz<0){nx=-nx;ny=-ny;nz=-nz;}
      const sh=0.32+0.6*Math.max(0,nx*L.x+ny*L.y+nz*L.z)+0.12*(nz*0.5+0.5);
      let r=br,gg=bg,b=bb;if(col){r=col.getX(i0);gg=col.getY(i0);b=col.getZ(i0);}
      const R=Math.min(255,(lin2s(Math.min(1,r*sh))+em*0.12)*255)|0,G=Math.min(255,(lin2s(Math.min(1,gg*sh))+em*0.43)*255)|0,B=Math.min(255,(lin2s(Math.min(1,b*sh))+em*0.92)*255)|0;
      const x0=sv[i0*3],y0=sv[i0*3+1],x1=sv[i1*3],y1=sv[i1*3+1],x2=sv[i2*3],y2=sv[i2*3+1];
      const area=(x1-x0)*(y2-y0)-(x2-x0)*(y1-y0);if(Math.abs(area)<1e-12)continue;
      const minx=Math.max(0,Math.floor(Math.min(x0,x1,x2))),maxx=Math.min(W-1,Math.ceil(Math.max(x0,x1,x2)));
      const miny=Math.max(0,Math.floor(Math.min(y0,y1,y2))),maxy=Math.min(H-1,Math.ceil(Math.max(y0,y1,y2)));
      if(minx>maxx||miny>maxy)continue;
      const ia=1/area;
      for(let y=miny;y<=maxy;y++){
        const py=y+0.5;
        for(let x=minx;x<=maxx;x++){
          const qx=x+0.5;
          const w0=((x1-qx)*(y2-py)-(x2-qx)*(y1-py))*ia,w1=((x2-qx)*(y0-py)-(x0-qx)*(y2-py))*ia,w2=1-w0-w1;
          if(w0<0||w1<0||w2<0)continue;
          if(cut&&(w0*cd[0]+w1*cd[1]+w2*cd[2]<0||(cut>1&&w0*cd[3]+w1*cd[4]+w2*cd[5]<0)))continue;
          const z=w0*z0+w1*z1+w2*z2,k=y*W+x;
          if(z>=zb[k])continue;
          if(op<1){const o=px[k],a=op;px[k]=0xff000000|((((o>>16)&255)*(1-a)+B*a)<<16)|((((o>>8)&255)*(1-a)+G*a)<<8)|(((o&255)*(1-a)+R*a)|0);}
          else{zb[k]=z;px[k]=0xff000000|(B<<16)|(G<<8)|R;}
        }
      }
    }
  }
  ctx.putImageData(img,0,0);
  // corner triad
  const S=110,cx=S/2,cy=H-22-S/2,q=cam.quaternion.clone().invert();
  for(const [t,v,c] of [['X',[1,0,0],'#b42318'],['Y',[0,1,0],'#166534'],['Z',[0,0,1],'#1d4ed8']]){
    const d=new THREE.Vector3(...v).applyQuaternion(q);ctx.strokeStyle=c;ctx.fillStyle=c;ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+d.x*36,cy-d.y*36);ctx.stroke();ctx.font='bold 16px system-ui';ctx.fillText(t,cx+d.x*46-5,cy-d.y*46+6);
  }
  ctx.fillStyle='#374151';ctx.font='14px system-ui';ctx.fillText('software preview: this browser has no WebGL',W-330,H-10);
}

// ---- render loop (renders only when something changed) ----
let lastDirty=0;
(function loop(now){
  requestAnimationFrame(loop);
  if(tween){
    const k=Math.min(1,(now-tween.t0)/280),s=k*k*(3-2*k);
    controls.target.lerpVectors(tween.from.t,tween.to.t,s);cam.position.lerpVectors(tween.from.p,tween.to.p,s);
    cam.zoom=tween.from.z+(tween.to.z-tween.from.z)*s;cam.updateProjectionMatrix();dirty=true;if(k>=1)tween=null;
  }
  controls.update();if(camMoved()){dirty=true;ctrlTrue++;}
  hover(now||0);
  if(!dirty)return;
  if(SOFT){if(!lastDirty){lastDirty=now;return;}if(now-lastDirty<60&&!tween)return;}   // the preview takes ~1 s: draw once settled
  dirty=false;lastDirty=0;renders++;
  if(SOFT){softRender();return;}
  if(fishOn()){fishRender();return;}
  renderer.setViewport(0,0,host.clientWidth,host.clientHeight);renderer.setScissorTest(false);
  renderer.clear();renderer.render(scene,cam);
  // corner triad, bottom left
  const S=110;renderer.clearDepth();renderer.setScissorTest(true);renderer.setScissor(0,22,S,S);renderer.setViewport(0,22,S,S);
  const d=cam.position.clone().sub(controls.target).normalize();tCam.position.copy(d).multiplyScalar(4);tCam.up.copy(cam.up);tCam.lookAt(0,0,0);
  renderer.render(tScene,tCam);renderer.setScissorTest(false);
})();
