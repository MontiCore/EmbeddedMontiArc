<template>
  <form class="d-flex flex-column border align-items-start gap-3" style="width: 30rem; border-color: white!important">
    <h3>Metadata</h3>
    <div class="text-start w-100">
      <label class="form-label" for="title">Title</label>
      <input class="form-control" type="text" id="title" @input="this.$emit('titleChanged', $event.target.value)"/>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="description">Description</label>
      <textarea class="form-control" id="description" style="height: 13.5rem"  @input="this.$emit('descriptionChanged', $event.target.value)"/>
    </div>
    <div class="text-start w-100">
      <label class="form-label" for="price">Price</label>
      <div class="input-group">
        <input class="form-control" type="number" id="price" step="0.01"  @input="this.$emit('priceChanged', $event.target.value)"/>
        <span class="input-group-text">€</span>
      </div>
    </div>
  </form>
</template>

<script>

export default {
  props: ['title', 'description', 'price']
}
</script>
