export function path_result(arr) {
    if(!window.astar) {
        return;
    }
    let old = window.astar.result;
    if(old.buffer.byteLength < arr.length * 4) {
        old = new Uint32Array(arr.length);
    }else{
        old = new Uint32Array(old.buffer);
    }
    old.set(arr);
    window.astar.result = old.subarray(0, arr.length);
}