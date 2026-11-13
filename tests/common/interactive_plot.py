## SPDX-License-Identifier: LGPL-2.1
## @file interactive_plot.py
## @brief Interactive review figures (uPlot, vendored in tests/common/vendor/): zoom in time, read exact values.
## @par In the page
##   Drag across the plot to zoom in time (the value axis fits what is visible); double-click to
##   reset; hover to read every signal at that instant (legend, 6 significant digits). The panels
##   of one figure zoom and hover together.
## @par Conventions (procedure §8.5)
##   Judged signal thick blue; reference thin grey (another run: dashed grey); limits dashed;
##   thresholds dash-dot; judged window shaded; events dotted vertical lines; samples outside a
##   limit as red dots; lower panels one unit each.
## @par Data
##   Embedded in the page (time float64, values float32, base64), drawn one figure at a time. Traces longer than MAX_POINTS
##   are reduced by keeping each bucket's min and max, so peaks survive; the caption says so.
"""Builds the figure data and the HTML that draws it; the page needs assets() once."""
from pathlib import Path
import base64,hashlib,html,json
import numpy as np

HERE=Path(__file__).resolve().parent
VENDOR=HERE/'vendor'
MAX_POINTS=200_000
SERIES=['#2a78d6','#eb6834','#1baf7a','#eda100']
INK2,MUTED,CRITICAL='#52514e','#8a8984','#e34948'


def reduce(t,y,n=MAX_POINTS):
    """@brief Keep min and max of each bucket when a trace is too long. @return (t, y, reduced?)."""
    t=np.asarray(t,dtype=float);y=np.asarray(y,dtype=float)
    if len(t)<=n:return t,y,False
    b=n//2;edges=np.linspace(0,len(t),b+1).astype(int);it=[];iy=[]
    for a,z in zip(edges[:-1],edges[1:]):
        seg=y[a:z]
        if not len(seg):continue
        i,j=a+int(np.nanargmin(seg)),a+int(np.nanargmax(seg))
        for k in sorted({i,j}):it.append(t[k]);iy.append(y[k])
    return np.asarray(it),np.asarray(iy),True


class Figure:
    """@brief One figure: a main chart and optional panels sharing the time axis."""
    def __init__(self,time_unit,xlabel=None):
        """@param time_unit unit of the x axis; xlabel replaces 'time (unit)' for a non-time x axis."""
        self.arrays=[];self._ids={};self.charts=[];self.time_unit=time_unit;self.xlabel=xlabel;self.reduced=False

    def _put(self,a,dtype):
        a=np.ascontiguousarray(a,dtype=dtype)
        key=(a.dtype.str,hashlib.sha1(a.tobytes()).hexdigest())  # a time base shared by several traces is stored once
        if key in self._ids:return self._ids[key]
        self.arrays.append({'d':'f8' if dtype==np.float64 else 'f4','b':base64.b64encode(a.tobytes()).decode()})
        self._ids[key]=len(self.arrays)-1
        return self._ids[key]

    def chart(self,unit,height):
        c={'unit':unit,'h':height,'series':[],'hlines':[],'window':None,'events':[]}
        self.charts.append(c);return c

    def line(self,chart,label,t,y,color,width=1.5,dash=None,points=False):
        t,y,red=reduce(t,y);self.reduced|=red
        y=np.where(np.isfinite(y),y,np.nan)
        chart['series'].append({'label':label,'t':self._put(t,np.float64),'y':self._put(y,np.float32),
                                'color':color,'w':width,'dash':dash,'points':points})

    def html(self,fid,alt):
        data=json.dumps({'unit':self.time_unit,'xlabel':self.xlabel,'arrays':self.arrays,'charts':self.charts},separators=(',',':'))
        return (f'<div class="iplot" data-fig="{html.escape(fid)}" role="img" aria-label="{html.escape(alt)}">'
                f'<div class="iplot-hint">Drag to zoom · double-click to reset · hover for values</div></div>'
                f'<script type="application/json" id="data-{html.escape(fid)}">{data.replace("</","<\\/")}</script>')


def assets():
    """@brief <style> and <script> the page needs once (library inlined: works offline)."""
    css=(VENDOR/'uPlot.min.css').read_text();js=(VENDOR/'uPlot.iife.min.js').read_text()
    return f'''<style>{css}
.iplot{{margin:6px 0;background:#fff;border:1px solid #e4e2dc;border-radius:6px;padding:6px 4px 2px;min-height:120px}}
.iplot-hint{{font-size:11px;color:{MUTED};padding:0 6px}} .iplot .u-legend{{font-size:11px;text-align:left}}
.iplot .u-legend .u-value{{font-family:ui-monospace,monospace}} .iplot .u-legend th{{font-weight:500}}</style>
<script>{js}</script>
<script>{RENDER_JS}</script>'''


RENDER_JS=r'''
(function(){
function dec(a){const s=atob(a.b),u=new Uint8Array(s.length);for(let i=0;i<s.length;i++)u[i]=s.charCodeAt(i);
 return a.d==='f8'?new Float64Array(u.buffer):new Float32Array(u.buffer);}
function fmt(v){return v==null||Number.isNaN(v)?'—':(Math.abs(v)>=1e6||(v!==0&&Math.abs(v)<1e-3))?v.toExponential(5):Number(v.toPrecision(6)).toString();}
function draw(el){
 const spec=JSON.parse(document.getElementById('data-'+el.dataset.fig).textContent);
 const arr=spec.arrays.map(dec);const plots=[];const key='k'+el.dataset.fig;
 const width=()=>Math.max(280,el.clientWidth-10);
 let syncing=false;
 spec.charts.forEach((c,ci)=>{
  // align series that have different time bases
  const tables=[],map=[];const byT={};
  c.series.forEach((s,si)=>{if(!(s.t in byT)){byT[s.t]=tables.length;tables.push([arr[s.t]]);}
    const tb=tables[byT[s.t]];tb.push(arr[s.y]);map.push([byT[s.t],tb.length-1]);});
  let data;
  if(tables.length===1)data=tables[0];else{data=uPlot.join(tables.map(t=>t.map(a=>Array.from(a))));}
  // column index in data for each series
  let offs=[],o=1;tables.forEach(t=>{offs.push(o);o+=t.length-1;});
  const order=c.series.map((s,si)=>offs[map[si][0]]+map[si][1]-1);
  // uPlot marks gaps with null; NaN (no sample, or not outside the limit) would break its ranges
  const gaps=a=>{for(let i=0;i<a.length;i++)if(Number.isNaN(a[i]))return Array.from(a,v=>Number.isNaN(v)?null:v);return a;};
  const cols=[data[0]];order.forEach(i=>cols.push(gaps(data[i])));
  const t0=cols[0][0],t1=cols[0][cols[0].length-1];
  const xl=spec.xlabel||('time ('+spec.unit+')');
  const series=[{label:xl,value:(u,v)=>v==null?'—':Number(v.toPrecision(8))+(spec.xlabel?'':' '+spec.unit)}].concat(c.series.map(s=>({
    label:s.label,stroke:s.points?'transparent':s.color,width:s.w,dash:s.dash||undefined,spanGaps:!s.points,
    paths:s.points?(()=>null):undefined,
    points:s.points?{show:true,size:6,fill:s.color,stroke:s.color}:{show:false},value:(u,v)=>fmt(v)})));
  const hl=c.hlines.map(h=>h.v);
  const opts={width:width(),height:c.h,series,
   legend:{live:true},
   cursor:{sync:{key,setSeries:false},drag:{x:true,y:false,setScale:true}},
   scales:{x:{time:false},y:{range:(u,mn,mx)=>{
     const zoomed=u.scales.x&&u.scales.x.min!=null&&(u.scales.x.min>t0+1e-12||u.scales.x.max<t1-1e-12);
     if(!zoomed)hl.forEach(v=>{mn=Math.min(mn,v);mx=Math.max(mx,v);});
     if(mn==null||mx==null)return[0,1];if(mn===mx){mn-=1;mx+=1;}const p=(mx-mn)*0.06;return[mn-p,mx+p];}}},
   axes:[{stroke:'#52514e',grid:{stroke:'#e4e2dc',width:1},ticks:{stroke:'#e4e2dc'},label:ci===spec.charts.length-1?xl:undefined,size:ci===spec.charts.length-1?44:30},
         {stroke:'#52514e',grid:{stroke:'#e4e2dc',width:1},ticks:{stroke:'#e4e2dc'},label:c.unit,size:70,values:(u,v)=>v.map(fmt)}],
   hooks:{
    drawClear:[u=>{const x=u.ctx,b=u.bbox;
      if(c.window){const a=u.valToPos(c.window[0],'x',true),z=u.valToPos(c.window[1],'x',true);
        x.save();x.fillStyle='#ebe9e3';x.fillRect(Math.max(a,b.left),b.top,Math.min(z,b.left+b.width)-Math.max(a,b.left),b.height);x.restore();}}],
    draw:[u=>{const x=u.ctx,b=u.bbox,dpr=devicePixelRatio||1;x.save();x.lineWidth=dpr;
      c.hlines.forEach(h=>{const y=u.valToPos(h.v,'y',true);if(y<b.top||y>b.top+b.height)return;
        x.strokeStyle=h.color;x.setLineDash(h.dash.map(d=>d*dpr));x.beginPath();x.moveTo(b.left,y);x.lineTo(b.left+b.width,y);x.stroke();});
      c.events.forEach((ev,i)=>{const px=u.valToPos(ev.t,'x',true);if(px<b.left||px>b.left+b.width)return;
        x.strokeStyle='#8a8984';x.setLineDash([2*dpr,3*dpr]);x.beginPath();x.moveTo(px,b.top);x.lineTo(px,b.top+b.height);x.stroke();
        if(ci===0&&ev.label){x.setLineDash([]);x.fillStyle='#52514e';x.font=(11*dpr)+'px system-ui,sans-serif';x.fillText(ev.label,px+4*dpr,b.top+(12+12*(i%2))*dpr);}});
      x.restore();}],
    setScale:[(u,k)=>{if(k!=='x'||syncing)return;syncing=true;
      plots.forEach(p=>{if(p!==u)p.setScale('x',{min:u.scales.x.min,max:u.scales.x.max});});syncing=false;}]}};
  const box=document.createElement('div');el.appendChild(box);
  const u=new uPlot(opts,cols,box);plots.push(u);
  u.over.addEventListener('dblclick',()=>plots.forEach(p=>p.setScale('x',{min:t0,max:t1})));
 });
 window.addEventListener('resize',()=>plots.forEach(p=>p.setSize({width:width(),height:p.height})));
}
// A figure drawn inside a closed <details> got the minimum width: refit when one opens (toggle does not bubble).
document.addEventListener('toggle',()=>window.dispatchEvent(new Event('resize')),true);
// One figure at a time, yielding to the browser in between: the page stays responsive while
// a long report decodes its data, and every figure is drawn whether or not it was scrolled to.
const els=[...document.querySelectorAll('.iplot')];
(function next(i){if(i>=els.length)return;try{draw(els[i])}catch(x){els[i].textContent='Plot failed: '+x}setTimeout(()=>next(i+1),0);})(0);
})();
'''
