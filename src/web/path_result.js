export function path_result(arr) {
    if(!window.astar) {
        return;
    }
    for(let i = arr.length - 1; i >= 0; i-= 2) {
        window.astar.result.push(arr[i]);
        window.astar.result.push(arr[i+1]);
    }
}