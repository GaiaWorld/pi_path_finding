export function path_result(arr) {
    let old = window._astar_path_result;
    if(old && old.buffer.byteLength >= arr.length * 4) {
        old = new Uint32Array(old.buffer);
    }else{
        old = new Uint32Array(arr.length);
    }
    old.set(arr);
    window._astar_path_result = old.subarray(0, arr.length);
}