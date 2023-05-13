export function path_result(arr) {
    let old = window._astar_path_result;
    if(old && old.buffer.byteLength >= arr.length * 4) {
        old = new Int32Array(old.buffer);
    }else{
        old = new Int32Array(arr.length);
    }
    old.set(arr);
    window._astar_path_result = old.subarray(0, arr.length);
}
export function round_result(arr) {
    let old = window._find_round_result;
    if(old && old.buffer.byteLength >= arr.length * 4) {
        old = new Int32Array(old.buffer);
    }else{
        old = new Int32Array(arr.length);
    }
    old.set(arr);
    window._find_round_result = old.subarray(0, arr.length);
}