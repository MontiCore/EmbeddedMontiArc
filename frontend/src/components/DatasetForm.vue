<template>
  <form class="d-flex flex-column p-3 border align-items-start gap-3" style="width: 30rem; border-color: white!important">
    <h3>Metadata</h3>
    <div class="text-start w-100">
      <label class="form-label" for="title">Title</label>
      <input class="form-control" type="text" id="title"/>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="description">Description</label>
      <textarea class="form-control" id="description" style="height: 13.5rem"/>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="price">Price</label>
      <div class="input-group">
        <input class="form-control" type="number" id="price" step="0.01"/>
        <span class="input-group-text">€</span>
      </div>
    </div>
    <button type="submit" class="btn btn-primary fw-bold">Create dataset</button>
  </form>
</template>

<script>

export default {
  data () {
    return {
    }
  },
  methods: {
  }
}
</script>
